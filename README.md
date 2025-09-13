# Flying ADC BMS
This BMS senses up to 16 channels per board. Boards can be stacked to sense any number of cells. Each board has a current sensor input therefor the current sensor can be located where it is most convenient. In addition each board has two temperature sensor inputs.

For cell voltage sensing a single channel, high precision delta/sigma ADC is used in conjunction with a 16 channel multiplexer. Each cell is sensed with a differential input with an accuracy 0.5 mV. The ADC is paired with a full bridge connected to its 5V isolated power supply. This allows sending current into a low cell and also draining current from a high cell. Only small balancing currents are used over a long time thus omitting large power resistors.
Due to its architecture the cell inputs are somewhat tolerant to miswiring, i.e. a swapped cell lead or multiple cells across one input won't damage the circuitry and will instead be reported by software.

# Wiring
Each board has 17 inputs for cell taps labeled V0 to V16. The very first input of the first board is connected to B-, i.e. the negative pole of the most negative cell. V16 of the first board is connected to the positive pole of the 16th cell. The latter is also connected to V0 of the second board.
Not all channels of a board need to be used if it is more convenient

A common CAN bus and 12V supply is connected to each module. In addition each module has an enable input an output. The first enable input must be connected to 12V whenever you want the BMS to run. subsequent enable inputs are connected to the enable output of the previous module.

Further documentation can be found here: https://openinverter.org/wiki/16-cell_BMS

# OTA (over the air upgrade)
The firmware is linked to leave the 4 kb of flash unused. Those 4 kb are reserved for the bootloader
that you can find here: https://github.com/jsphuebner/stm32-CANBootloader/

When flashing your device for the first time you must first flash that bootloader. After that you can
use the ESP32 CAN module and its web interface to upload your actual application firmware.

The web interface is here: https://github.com/jsphuebner/esp32-web-interface/tree/can-backend

Binaries are here: https://github.com/jsphuebner/FlyingAdcBms/actions

# Compiling
You will need the arm-none-eabi toolchain: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads
On Ubuntu type

`sudo apt-get install git gcc-arm-none-eabi`

The only external depedencies are libopencm3 and libopeninv. You can download and build these dependencies by typing

`make get-deps`

Now you can compile stm32-<yourname> by typing

`make`

And upload it to your board using a JTAG/SWD adapter, the updater.py script or the esp8266 web interface.

# Editing
The repository provides a project file for Code::Blocks, a rather leightweight IDE for cpp code editing.
For building though, it just executes the above command. Its build system is not actually used.
Consequently you can use your favority IDE or editor for editing files.

# Configuration parameters (advanced)
The firmware exposes configuration parameters via the parameter system (see `include/param_prj.h`).
Below are advanced parameters related to balancing focus mode, target computation, and older hardware timing.

- balFocusEnable (0/1, default 1)
  Enables the Focused Weak-Cell mode. When enabled, the BMS can hold on the weakest cell(s) for long durations to accelerate recovery.

- balFocusMinDev (mV, default 15)
  Minimum deviation below the balance target required for a cell to qualify for the focus set.

- balFocusTopN (1..4, default 1)
  Number of cells to include in the focus set (typically 1). The most deviating cell becomes the active focus channel.

- balFocusHold (s, default 600)
  Hold time per focused cell. During this time, the channel is pinned to the focused cell to maximize balancing duty.

- balAllowRun (0/1, default 1)
  Allows balancing while in RUN if |idcavg| is below the `idlecurrent` threshold, effectively treating low-current RUN like IDLE.

- balTargetTrimLowK (cells, default 1)
  Trims the balance target by excluding the K lowest cells when computing the target. This prevents one weak cell from pulling the target down and causing unnecessary discharge of already-balanced cells. Note: this affects the target used for balancing decisions only; the displayed `uavg/umin/umax` remain untrimmed.
  Set to 0 to revert to legacy behavior.

- balFocusRefresh (s, default 60)
  While focusing a weak cell, perform a full periodic sweep every N seconds to refresh `uavg/umin/umax` and re-evaluate the focus set.

- muxSettleExtra (ms, default 6; effective on older HW only)
  Adds extra settle time between MUX channel select and ADC start to prevent repeated identical readings on older boards. Applied only on HW_20/HW_21/HW_22/HW_24; ignored on HW_23 and HWV1.

Notes:
- When focus mode is active, the focused cell is never discharged (charge-only on that channel), and the `ubalance` gate is bypassed so weak cells can be raised even when average voltage is low.
- The trimmed target affects balancing decisions (charge/discharge thresholding) but does not alter the reported `uavg/umin/umax` values.

# Full parameter reference
This section documents all parameters declared in `include/param_prj.h`.

Parameters are grouped by category. Each bullet lists:
- name (unit, range, default): brief description

## BMS (CAT_BMS)
- `gain` (mV/dig, 1–1000, 587): ADC scaling factor to convert ADC digits to millivolts.
- `correction0` (ppm, -10000–10000, 1800): PPM calibration applied on channel 0.
- `correction1` (ppm, -10000–10000, 3700): PPM calibration applied on channel 1.
- `correction15` (ppm, -10000–10000, 1000): PPM calibration applied on channel 15.
- `numchan` (, 1–16, 12): Number of active cell channels on this board.
- `balmode` (BALMODE, 0–3, 0): 0=Off, 1=Additive (charge low cells), 2=Dissipative (discharge high cells), 3=Both.
- `ubalance` (mV, 0–4500, 4500): Balancing allowed only when average cell voltage exceeds this threshold (focus mode can bypass).
- `balDeltaMed` (cycles, 0–500, 150): Extra balancing cycles when deviation >15 mV (legacy sweep mode).
- `balDeltaHigh` (cycles, 0–750, 300): Extra balancing cycles when deviation >30 mV (legacy sweep mode).
- `safeVoltage` (mV, 3000–4200, 4000): Above this, extended cycles are reduced to protect high-voltage operation.
- `maxScanDelay` (s, 5–60, 30): Caps total extra balancing time per sweep across cells.
- `balFocusEnable` (0/1, 0–1, 1): Enables Focused Weak-Cell mode (long hold on weakest cells).
- `balFocusMinDev` (mV, 0–500, 15): Minimum deficit to enter focus set.
- `balFocusTopN` (, 1–4, 1): Number of weakest cells to consider; the worst becomes the current focus.
- `balFocusHold` (s, 0–36000, 600): Hold time per focused cell.
- `balAllowRun` (0/1, 0–1, 1): Allow balancing in RUN when |`idcavg`| < `idlecurrent` (treat low-current RUN like IDLE).
- `balTargetTrimLowK` (cells, 0–4, 1): Exclude K lowest cells when computing balance target (prevents one weak cell from dragging target down). Set 0 for legacy.
- `balFocusRefresh` (s, 0–3600, 60): Periodic full sweep while focusing to refresh `uavg/umin/umax` and re-evaluate focus set.
- `muxSettleExtra` (ms, 0–50, 6): Extra settle delay between MUX select and ADC start. Applied on older HW_20/21/22/24.
- `idlewait` (s, 0–100000, 60): Idle transition time; if low current persists this long in RUN, FSM enters IDLE.
- `sleeptimeout` (h, 0–99, 2): Time in IDLE before disabling self/next enable (auto-sleep) when not enabled.
- `idlecurrent` (mA, 0–9999, 800): Threshold separating RUN vs idle-like operation (used by FSM and balancing gates).

## Battery Characteristics (CAT_BAT)
- `dischargemax` (A, 1–2047, 200): Max discharge current. Used to compute `dischargelim`.
- `nomcap` (Ah, 0–1000, 100): Nominal capacity for SOC/SOH calculations.
- `icc1` (A, 1–2000, 70): CC current segment 1 for charge algorithm.
- `icc2` (A, 1–2000, 50): CC current segment 2 for charge algorithm.
- `icc3` (A, 1–2000, 20): CC current segment 3 for charge algorithm.
- `ucv1` (mV, 3000–4500, 3900): CV voltage threshold 1.
- `ucv2` (mV, 3000–4500, 4000): CV voltage threshold 2.
- `ucellmax` (mV, 1000–4500, 4200): Cell max limit.
- `ucellmin` (mV, 1000–4500, 3300): Cell min limit.
- `ucell0soc` … `ucell100soc` (mV, 2000–4500, defaults 3300..4200): Voltage-to-SOC lookup points at 0,10,…,100%.
- `sohpreset` (%, 10–100, 100): Preset SoH value; used to initialize/override SoH.

## Sensor setup (CAT_SENS)
- `idcgain` (dig/A, -1000–1000, 10): Current sensor gain for conversion.
- `idcofs` (dig, -4095–4095, 0): Current sensor offset.
- `idcmode` (IDCMODES, 0–3, 0): 0=Off, 1=AdcSingle, 2=AdcDifferential, 3=IsaCan.
- `tempsns` (TEMPSNS, 0–3, 0): 0=None, 1=Chan1, 2=Chan2, 3=Both temperature sensors.
- `tempres` (Ohm, 10–500000, 10000): Thermistor nominal resistance.
- `tempbeta` (, 1–100000, 3900): Thermistor beta parameter.

## Communication (CAT_COMM)
- `pdobase` (, 0–2047, 500): Base CAN PDO identifier for module data.
- `sdobase` (, 0–63, 10): Base CAN SDO node ID.

## Testing (CAT_TEST)
- `enable` (0/1, 0–1, 1): Global enable. When 0, BMS remains disabled or enters ERROR; used across the system.
- `testchan` (, -1–15, -1): If >=0, reads a single channel in test mode.
- `testbalance` (BALMODE, 0–2, 0): Test balance command (0=None,1=Discharge,2=ChargePos/Neg per channel parity).

## VX1 general settings (CAT_VX1)
- `VX1mode` (0/1, 0–1, 1): Enable VX1 integration features.

## VX1 Charger settings (CAT_VX1_CHR)
- `VX1chrCellMaxV` (mV, 3800–4200, 4150): Target max cell voltage for charger coordination.

## VX1 CAN settings (CAT_VX1_CAN)
- `VX1SendConfigMsg` (0/1, 0–1, 0): Send VX1 configuration messages.
- `VX1enCanMsg` (0/1, 0–1, 1): Enable periodic VX1 BMS CAN messages.
- `VX1BootLCDMsg` (0/1, 0–1, 1): Show boot message on the VX1 LCD.
- `VX1enBootstats` (0/1, 0–1, 1): Send boot statistics on startup.
- `VX1msgInterval` (ms, 50–1000, 100): Base interval for VX1 message scheduling.
- `VX1LCDClockStats` (0/1/2, 0–2, 1): 0=Off, 1=Always, 2=Idle-only display of clock stats.
- `VX1LCDClockStatVal` (0–6, 0–6, 2): Select value for LCD clock stats (0=soc, 1=uavg, 2=udelta, 3=tempmax, 4=power, 5=idcavg, 6=kWh/100km).
- `VX1ErrWarn` (0/1, 0–1, 1): Enable VX1 error/warning reporting.
- `VX1TempWarn` (0/1, 0–1, 1): Enable VX1 temperature warning logic.
- `VX1TempWarnTest` (0/1, 0–1, 0): Force temperature warning test mode.
- `VX1uDeltaWarn` (0/1, 0–1, 1): Enable VX1 delta‑voltage warning.
- `VX1uDeltaWarnTresh` (mV, 2–500, 150): Threshold for delta‑voltage warning.
- `VX1uDeltaWarnTest` (0/1, 0–1, 0): Force delta‑voltage warning test mode.
- `VX1EmulateBMSmsg` (0/1, 0–1, 1): Emulate OEM BMS messages for VX1 compatibility.
- `VX1kWhResetDist` (km, 0.1–20, 5): Distance threshold to reset kWh/100km average.
- `VX1TempWarnHiPoint` (°C, 40–80, 55): High temperature warning threshold.
- `VX1TempWarnLoPoint` (°C, -25–40, -1): Low temperature warning threshold.
- `VX1mockTemp` (°C, -20–55, 24): Mock temperature value for testing.
- `VX1SendFirmwareRevision` (0/1, 0–1, 0): Include firmware revision in VX1 messages.
- `VX1SendCellVoltages` (0/1, 0–1, 0): Include per-cell voltages in VX1 messages (format depends on implementation).

## Enum definitions
- `HWREVS`: 0=Unknown, 1=v1.x, 2=v2.0, 3=v2.1, 4=v2.2, 5=v2.3, 6=v2.4
- `OPMODES`: 0=Boot, 1=GetAddr, 2=SetAddr, 3=ReqInfo, 4=RecvInfo, 5=Init, 6=SelfTest, 7=Run, 8=Idle, 9=Error
- `OFFON`: 0=Off, 1=On
- `BALMODE`: 0=Off, 1=Additive, 2=Dissipative, 3=Both
- `BAL`: 0=None, 1=Discharge, 2=ChargePos, 3=ChargeNeg (status encoding)
- `IDCMODES`: 0=Off, 1=AdcSingle, 2=AdcDifferential, 3=IsaCan
- `TEMPSNS`: 0=None, 1=Chan1, 2=Chan2, 3=Both
- `VX1MODE`: 0=Off, 1=On

## Display values (read-only)
These are reported values, not user-configurable:
- `version` (VERSTR), `hwrev` (HWREVS), `opmode` (OPMODES)
- `lasterr`, `errinfo`
- `modaddr`, `modnum`, `totalcells`, `counter`, `uptime`
- `chargein` (As), `chargeout` (As), `soc` (%), `soh` (%)
- `chargelim` (A), `dischargelim` (A), `idc` (A), `idcavg` (A), `power` (W)
- `tempmin` (°C), `tempmax` (°C)
- `uavg` (mV), `umin` (mV), `umax` (mV), `udelta` (mV), `utotal` (mV)
- `u0` … `u15` (mV): Per-cell voltages for channels 0..15
- Per-module values (for daisy-chained systems): `uavg0/1/…`, `umin0/1/…`, `umax0/1/…`, `tempmin0/1/…`, `tempmax0/1/…`
- Balancing command status per cell: `u0cmd` … `u15cmd` (BAL)
- CPU load: `cpuload` (%)
- VX1 display/can values: `VX1speed`, `VX1busVoltage`, `VX1busCurrent`, `VX1kWhper100km`, `VX1DebugParam1`, `VX1DebugParam2`
- Motor controller temps (from PGN 0xFF05): `MCHeatsinkTemp`, `MCCapacitor1Temp/2Temp/3Temp`
- VX1-specific mirrors: `VX1umax`, `VX1umin`, `VX1utotal`, `VX1tempmin`, `VX1tempmax`, `VX1udelta`, `VX1uavg`
- MC status bits: `MCReady`, `MCBrakeRight`, `MCBrakeLeft`, `MCThrottleFwd`, `MCReverse`, `MCStatusRaw`

# Adding classes or modules
As your firmware grows you probably want to add classes. To do so, put the header file in include/ and the 
source file in src/ . Then add your module to the object list in Makefile that starts in line 43 with .o
extension. So if your files are called "mymodule.cpp" and "mymodule.h" you add "mymodule.o" to the list.

When changing a header file the build system doesn't always detect this, so you have to "make clean" and
then make. This is especially important when editing the "*_prj.h" files.
