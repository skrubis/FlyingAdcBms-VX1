# ISO 26262 Functional Safety Audit — FlyingAdcBms-VX1

**Date:** 2026-06-05
**Reviewer:** Claude Sonnet 4.6 (AI-assisted review, ISO 26262 perspective)
**Codebase:** FlyingAdcBms-VX1 @ commit `fd35342`
**Platform:** STM32F103 / libopencm3
**Language:** C++
**System:** Multi-module lithium BMS for VX1 electric scooter

> **Disclaimer from vx1.cpp:** *"This project is not certified for use in production vehicles or safety-critical systems. Use of this project is entirely at your own risk."*

---

## Scope

Full static review of all production source files:
`src/main.cpp`, `src/bmsio.cpp`, `src/bmsfsm.cpp`, `src/bmsalgo.cpp`,
`src/flyingadcbms.cpp`, `src/selftest.cpp`, `src/hwinit.cpp`,
`src/temp_meas.cpp`, `src/vx1.cpp`, `include/param_prj.h`, `include/bmsio.h`

---

## CRITICAL Findings

### CRIT-1: Overvoltage Hardware Cutoff Is Commented Out

**File:** `src/main.cpp:69-73`

```cpp
/*
   if (Param::GetFloat(Param::umax) < (Param::GetFloat(Param::ucellmax) - 50))
      DigIo::nextena_out.Set();
   else if (Param::GetFloat(Param::umax) >= Param::GetFloat(Param::ucellmax))
      DigIo::nextena_out.Clear();*/
```

The only hardware-level overvoltage response is completely commented out. The **sole remaining protection** is the `chargelim` value published over CAN at 100 ms intervals. This requires the charger to be CAN-connected, compliant, and responsive within 100 ms. If any of these conditions fail, there is no backup hardware disconnect. Lithium cells charged above their ceiling enter thermal runaway.

**Required fix:** Restore the commented block or implement an equivalent hard GPIO cutoff, independent of CAN, triggered within one scan cycle of overvoltage detection.

---

### CRIT-2: Independent Watchdog Is Reset But Never Started

**File:** `src/main.cpp:138`, `src/hwinit.cpp` (no `iwdg_start` anywhere)

```cpp
// Ms100Task — called every 100 ms:
iwdg_reset();
// ← iwdg_start() is never called anywhere in the codebase
```

`iwdg_reset()` is called but the STM32 IWDG is never initialized with `iwdg_start()`. A firmware hang — from a stuck bit-bang I2C loop, an infinite loop in `__cxa_pure_virtual`, or any other fault — will not trigger a reset. The system will remain in a frozen state indefinitely with the battery in whatever state it was when it hung.

**Required fix:** Call `iwdg_start(IWDG_PRE_xxx, reload_value)` during initialization with a timeout ≤ 200 ms.

---

### CRIT-3: No Main Pack Contactor Control

**File:** Architecture-level gap

The BMS controls two digital outputs: `selfena_out` (BMS self-hold) and `nextena_out` (next module enable). There is no output that controls a **main high-voltage contactor or disconnect relay**. In any fault condition, the BMS cannot physically disconnect the battery from the load or charger independently of CAN communication. The architecture assumes the inverter/charger is the only entity that can disconnect the pack.

**Required fix:** Drive a dedicated fail-safe contactor output from a latched fault flag. The latch must require deliberate operator action to clear.

---

### CRIT-4: Division by Zero — `nomcap = 0` Is a Valid Parameter

**File:** `include/param_prj.h:68`, `src/bmsalgo.cpp:36`

```cpp
// param_prj.h — minimum allowed value is 0:
PARAM_ENTRY(CAT_BAT, nomcap, "Ah", 0, 1000, 100, 9)

// bmsalgo.cpp — denominator can be zero:
float soc = lastSoc + (100 * asDiff / (3600 * nominalCapacity));
```

If `nomcap = 0`, `CalculateSocFromIntegration` divides by zero. IEEE 754 produces `±inf` or `NaN`, which propagates into CAN-transmitted chargelim/dischargelim values. A charger receiving `NaN` may interpret it as max or zero current depending on its driver.

**Required fix:** Set `nomcap` minimum to 1 Ah. Add a zero guard in `CalculateSocFromIntegration`.

---

## HIGH Severity Findings

### HIGH-1: No Low-Temperature Derating for Discharge

**File:** `src/main.cpp:65-68`

```cpp
float dischargeCurrentLimit = Param::GetFloat(Param::dischargemax);
dischargeCurrentLimit *= BmsAlgo::LimitMinimumCellVoltage(...);
dischargeCurrentLimit *= BmsAlgo::HighTemperatureDerating(Param::GetFloat(Param::tempmax), 53);
// ← No LowTemperatureDerating() — cold discharge is unrestricted
```

`LowTemperatureDerating()` exists and is correctly applied to charge current (0% below −20°C, 30% at 0°C). It is never applied to discharge. High-rate discharge at sub-zero temperatures causes lithium plating on Li-ion anodes, which can trigger internal shorts on subsequent charging.

**Required fix:** Apply `LowTemperatureDerating(tempmin)` to the discharge current limit.

---

### HIGH-2: Temperature Sensor Open-Circuit Failure Is Silent

**File:** `src/temp_meas.cpp:35`, `src/bmsio.cpp:556`

```cpp
// digit=0 (disconnected thermistor) → resistance = +∞ → returns +∞°C
float resistance = seriesResistor * (voltageRatio / (digit / maxAdcValue) - 1.0f);

// In Accumulate():
if (tempmin0 < NO_TEMP)  // NO_TEMP = 127 — +∞ < 127 → FALSE
// → temperature silently dropped from aggregate, protection disabled
```

An open-circuit thermistor returns `+∞`. The sentinel check treats this identically to "no sensor configured." An operator who set `tempsns=1` believes temperature protection is active; it is silently bypassed with no fault posted.

**Required fix:** Validate ADC counts before conversion. Post a fault and limit current to zero when a configured sensor reads out of valid range (e.g., < 10 digits or > 4085 digits).

---

### HIGH-3: Temperature Monitoring Disabled by Default

**File:** `include/param_prj.h:91`

```cpp
PARAM_ENTRY(CAT_SENS, tempsns, TEMPSNS, 0, 3, 0, 52)
//                                           ↑ default = 0 = "None"
```

The default configuration ships with no temperature sensors enabled. Charge and discharge proceed without any thermal derating unless the operator explicitly configures `tempsns`. ISO 26262 requires safe defaults for parameters that affect hazard avoidance.

**Required fix:** Change the default to 1 or 3, or prevent entry into RUN state until `tempsns` is explicitly set to a non-zero value.

---

### HIGH-4: Error State Recovers to RUN Without Re-Running Self-Test

**File:** `src/bmsfsm.cpp:202-205`

```cpp
case ERROR:
   if (Param::GetBool(Param::enable))
      return RUN;  // ← bypasses INIT → SELFTEST sequence
```

After a self-test failure, toggling `enable = 1` immediately transitions to RUN. The fault condition that caused ERROR may still be present. Additionally, `TestCellConnection` in `selftest.cpp:124-126` uses `static bool overVoltage` and `static bool polarityCheckComplete` — these retain values across calls, so a re-run would return the cached result without actually retesting.

**Required fix:** On recovery from ERROR, always transition INIT → SELFTEST → RUN. Reset all static state in `SelfTest` before re-running.

---

### HIGH-5: CAN Module Discovery (0x7DD) Has No Authentication

**File:** `src/bmsfsm.cpp:223-232`

```cpp
case 0x7dd:
   recvNodeId = data[1] & 0xFF;
   recvIndex  = (data[1] >> 8) & 0xFF;
   recvPdoBase = data[1] >> 16;
```

Any device on the CAN bus can send a 0x7DD frame and re-assign any module's node ID, index, and PDO base address. A faulty node or attacker could corrupt BMS topology: a submodule could be told it is the master, or the master could subscribe to non-existent submodules, causing stale-data reads for chargelim/dischargelim.

**Required fix:** Restrict 0x7DD acceptance to BOOT/GET_ADDR states only. Reject address assignments during RUN/IDLE.

---

### HIGH-6: Sub-Module Discovery Timeout Proceeds with Incomplete Data

**File:** `src/bmsfsm.cpp:131-143`

```cpp
case RECV_INFO:
   if (canSdo->SDOReadReply(sdoReply))
   {
      // ... update totalcells, numModules ...
      return infoIndex < MAX_SUB_MODULES ? REQ_INFO : INIT;
   }
   return INIT;  // ← timeout: proceeds with wrong totalcells, no error posted
```

If a submodule's SDO read does not reply in one 100 ms cycle, the system proceeds to INIT with `totalcells` set only to the modules discovered so far. `Accumulate()` then divides total voltage sum by the wrong cell count, producing incorrect `uavg`. No fault is posted, no retry is attempted.

**Required fix:** Implement a retry counter with a timeout error flag. Transition to ERROR state if a configured submodule fails to respond.

---

### HIGH-7: `ForceSOCFromVoltage` Is Dead Code with a Harmful Side Effect

**File:** `src/main.cpp:220-256`

```cpp
// Runs unconditionally at boot, before any ADC readings:
Param::SetInt(Param::errinfo, (int)umin);  // ← always clobbers fault info
```

Called at boot before any scheduler ticks (`umin = 0`). The `if (umin > 2500)` guard prevents the SOC logic from executing, but line 231 **unconditionally overwrites `errinfo`** with 0, destroying any fault information that might have been readable post-reboot. The rest of the function (double call to `CalculateSocSoh`, accumulator reset) is dead code that creates maintenance risk.

**Required fix:** Remove `ForceSOCFromVoltage` entirely. `LoadNVRAM()` correctly restores SoC from `BKP_DR1`.

---

## MEDIUM Severity Findings

### MED-1: Coulomb Counter Has No Overflow Saturation

**File:** `src/bmsio.cpp:483`

```cpp
chargein += amsIn / 200;   // no saturation — wraps at ~18,000 Ah accumulated
```

`chargein`/`chargeout` are signed 32-bit fixed-point (max ~18,000 Ah). At 200 A sustained average, wraparound occurs after ~91 hours. After wraparound, `asDiff` becomes a large negative number, corrupting SoC integration and SoH estimation silently.

**Required fix:** Saturate at max representable value, or periodically anchor integration to a voltage-estimated SoC.

---

### MED-2: ADC Conversion Completion Status Not Verified

**File:** `src/flyingadcbms.cpp:126-136`

The MCP3425 returns a config/status byte as its third read byte (bit 7 = conversion in progress). The code reads and discards this byte without checking it. If `StartAdc` is silently dropped due to the I2C lock, `GetResult` returns the previous conversion's value with no indication of failure.

**Required fix:** Check `data[2] & 0x80` in `GetResult()`. Return a sentinel and flag an error if conversion is still in progress.

---

### MED-3: `__cxa_pure_virtual` Spins Forever

**File:** `src/main.cpp:407`

```cpp
extern "C" void __cxa_pure_virtual() { while (1); }
```

Combined with the unstarted IWDG (CRIT-2), a pure virtual call hangs the system permanently with no recovery path.

**Required fix:** Call `NVIC_SystemReset()` or set a safe-state latch. Only effective once CRIT-2 is also fixed.

---

### MED-4: Soft Undervoltage Limit Only — No Hard Cutoff

**File:** `src/bmsalgo.cpp:140-146`, `src/main.cpp:66`

`LimitMinimumCellVoltage()` ramps discharge limit to zero over 50 mV above `ucellmin`. This is a CAN-communicated soft limit — the inverter must respect it within the 100 ms update cycle. A high-rate discharge can overshoot `ucellmin` by tens of millivolts before the next CAN update. There is no hard GPIO cutoff.

**Required fix:** Pair with a hard GPIO inverter-enable interlock triggered when `umin < ucellmin - hysteresis`.

---

### MED-5: `totalcells` Division Without Zero Guard

**File:** `src/bmsio.cpp:565`

```cpp
Param::SetFloat(Param::uavg, totalSum / Param::GetInt(Param::totalcells));
```

Flash parameter corruption or an unexpected FSM state could yield `totalcells = 0`, producing `NaN`/`±inf` in `uavg` and all derived values.

**Required fix:** `int tc = MAX(1, Param::GetInt(Param::totalcells));`

---

## Summary

| ID | Severity | File | Finding |
|---|---|---|---|
| CRIT-1 | **Critical** | main.cpp:69 | Overvoltage HW cutoff commented out |
| CRIT-2 | **Critical** | hwinit.cpp | IWDG never started |
| CRIT-3 | **Critical** | Architecture | No main contactor control output |
| CRIT-4 | **Critical** | bmsalgo.cpp:36 | Div/zero when nomcap=0 |
| HIGH-1 | High | main.cpp:66 | No cold-discharge derating |
| HIGH-2 | High | temp_meas.cpp:35 | Sensor open-circuit silently disables protection |
| HIGH-3 | High | param_prj.h:91 | tempsns defaults to None (no temperature protection) |
| HIGH-4 | High | bmsfsm.cpp:203 | ERROR→RUN bypasses self-test re-run |
| HIGH-5 | High | bmsfsm.cpp:228 | CAN 0x7DD address assignment unauthenticated |
| HIGH-6 | High | bmsfsm.cpp:142 | SDO timeout → wrong totalcells, no fault |
| HIGH-7 | High | main.cpp:231 | Dead code clobbers errinfo at every boot |
| MED-1 | Medium | bmsio.cpp:483 | Coulomb counter no overflow saturation |
| MED-2 | Medium | flyingadcbms.cpp:126 | ADC busy bit not checked in GetResult |
| MED-3 | Medium | main.cpp:407 | Pure virtual spins forever (no IWDG fallback) |
| MED-4 | Medium | bmsalgo.cpp:140 | Soft UVP only — no hard GPIO cutoff |
| MED-5 | Medium | bmsio.cpp:565 | totalcells not guarded before division |

---

## Architectural Observations

1. **Single measurement path, no redundancy.** All cell voltages pass through one bit-bang I2C bus, one multiplexer chain, one ADC. A silently-dropped I2C transaction returns stale data with no error indication (`SendRecvI2C:169` returns immediately on lock contention).

2. **Protection is entirely CAN-dependent.** The BMS is an advisory system — it tells the charger/inverter what limits to respect. It does not actively enforce limits via hardware interlocks. This requires the charger and inverter to be trusted, CAN-reliable, and safety-certified themselves.

3. **Focus-mode safety sweep complexity.** The balancing state machine (`bmsio.cpp:99-422`, ~300 lines) maintains several interacting state variables (`safetySweepActive`, `focusActive`, `safetyPrimed`, `safetyPrimeWaitCycles`, `lastRequestedMux`). If the sweep does not complete before focus re-activates, individual cell voltages used for overvoltage decisions may be multiple seconds stale.

4. **SoC lookup defaults tuned for NMC, not LFP.** The default `voltageToSoc[]` table (3300 mV = 0%, 4200 mV = 100%) and `ucellmin = 3300 mV` are suited to NMC chemistry. An LFP deployment without recalibrating every lookup point would see premature 0% SoC reports at ~50% actual capacity.

5. **VX1 CAN runs at lowest interrupt priority (0xF0).** Correct for non-safety traffic. However, heavy VX1 CAN activity could add latency to `canMapExternal->SendAll()` in `Ms100Task`, delaying updated chargelim delivery to the charger.

---

*This audit was produced by AI-assisted static analysis. It is not a substitute for formal ISO 26262 certification activities (FMEA, FTA, HARA, verification testing). All findings should be reviewed by a qualified functional safety engineer before use in any vehicle.*
