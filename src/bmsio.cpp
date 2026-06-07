/*
 * This file is part of the FlyingADCBMS project.
 *
 * Copyright (C) 2025 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "bmsio.h"
#include "params.h"
#include "anain.h"
#include "temp_meas.h"
#include "my_math.h"
#include "flyingadcbms.h"

BmsFsm* BmsIO::bmsFsm;
int BmsIO::muxRequest = -1;
int BmsIO::muxSettledChannel = -1;
int BmsIO::focusChan = -1;
int BmsIO::focusHoldCyclesRemaining = 0;
bool BmsIO::focusActive = false;

// Local helpers for trimmed target and min cell
static void GetMinCell(int numChan, int& minIdx, float& minVal)
{
   minIdx = 0;
   minVal = Param::GetFloat((Param::PARAM_NUM)(Param::u0 + 0));
   for (int i = 1; i < numChan; i++)
   {
      float u = Param::GetFloat((Param::PARAM_NUM)(Param::u0 + i));
      if (u < minVal) { minVal = u; minIdx = i; }
   }
}

static float ComputeTrimmedAvgExcludingMin(int numChan)
{
   int minIdx; float minVal;
   GetMinCell(numChan, minIdx, minVal);
   float sum = 0;
   for (int i = 0; i < numChan; i++)
   {
      if (i == minIdx) continue;
      sum += Param::GetFloat((Param::PARAM_NUM)(Param::u0 + i));
   }
   int count = (numChan > 1) ? (numChan - 1) : numChan;
   return (count > 0) ? (sum / count) : minVal;
}

/** \brief Mux control function. Must be called in 2 ms interval */
void BmsIO::SwitchMux()
{
   static int channel = -1;
   static int adcStartChannel = -1;
   static bool startAdc = false;
   // Ignore new mux requests while an ADC conversion is in flight to prevent
   // turning the mux off mid-conversion. Keep this below the 25ms read period
   // minus the mux-off/select/start sequence, or the next ADC result can be
   // stale and signed with the new channel polarity.
   static int conversionHoldoffTicks = 0; // 2ms ticks; 9 => ~18ms

   if (conversionHoldoffTicks > 0)
   {
      conversionHoldoffTicks--;
   }

   //t=0 ms: On a mux change request first completely turn off mux
   if (muxRequest >= 0 && conversionHoldoffTicks == 0)
   {
      FlyingAdcBms::MuxOff();
      muxSettledChannel = -1;
      channel = muxRequest;
      muxRequest = -1;
   }
   //t=2 ms: switch to requested channel
   else if (channel >= 0)
   {
      adcStartChannel = channel;
      FlyingAdcBms::SelectChannel((uint8_t)adcStartChannel);
      channel = -1;
      startAdc = true;
   }
   //t=4 ms: start ADC
   else if (startAdc)
   {
      FlyingAdcBms::StartAdc();
      muxSettledChannel = adcStartChannel;
      if (muxRequest == adcStartChannel)
         muxRequest = -1;
      adcStartChannel = -1;
      startAdc = false;
      // Hold off accidental mux changes during the ADC conversion, but still
      // allow the next requested channel to start immediately after the 25ms read.
      conversionHoldoffTicks = 9; // 9 * 2ms = 18ms
   }
   //t=21 ms: ADC conversion is finished
   //t=25 ms: ADC conversion result is read
}

void BmsIO::MuxOff()
{
   muxRequest = -1;
   muxSettledChannel = -1;
   FlyingAdcBms::MuxOff();
}

void BmsIO::ReadCellVoltages()
{
   const int totalBalanceCycles = 30;
   static uint8_t chan = 0, balanceCycles = 0;
   static float sum = 0, min = 8000, max = 0;
   int balMode = Param::GetInt(Param::balmode);
   // Focus dwell safety sweep interval (seconds) is configurable via balFocusSweep
   int safetyIntervalSec = Param::GetInt(Param::balFocusSweep);
   if (safetyIntervalSec < 1) safetyIntervalSec = 1; // clamp safeguards
   static int safetyCountdownCycles = -1; // 25ms cycles remaining until next sweep
   static bool safetySweepActive = false;
   static int safetyReadCount = 0;
   static int prevSafetyIntervalSec = -1;
   static bool safetyPrimed = false; // true after we've kicked off ADC for chan 0 at sweep start
   static int safetyPrimeWaitCycles = 0; // number of 25ms cycles to skip before first sweep sample
   // Gate repeated mux requests during focus dwell so we don't spam the mux
   static int lastRequestedMux = -1;
   if (prevSafetyIntervalSec != safetyIntervalSec || safetyCountdownCycles < 0)
   {
      safetyCountdownCycles = (safetyIntervalSec * 1000) / 25;
      prevSafetyIntervalSec = safetyIntervalSec;
   }

   // Continuous focus selection using stored u-values (these update at safety sweeps)
   if (Param::GetInt(Param::balFocusEnable) == 1)
   {
      int numChan = Param::GetInt(Param::numchan);
      int minIdx; float minVal;
      GetMinCell(numChan, minIdx, minVal);
      float trimmedAvg = ComputeTrimmedAvgExcludingMin(numChan);
      float deficit = trimmedAvg - minVal;

      if (focusHoldCyclesRemaining > 0)
      {
         focusHoldCyclesRemaining--;
      }

      if (deficit >= Param::GetFloat(Param::balFocusMinDev))
      {
         // Start/refresh focus on the current lowest cell
         if (!focusActive || focusChan != minIdx)
         {
            focusChan = minIdx;
            focusActive = true;
         }
         // Refresh/extend hold
         focusHoldCyclesRemaining = Param::GetInt(Param::balFocusHold) * (1000 / 25);
      }
      else if (focusHoldCyclesRemaining <= 0)
      {
         // No longer meets deficit and hold expired -> leave focus
         focusActive = false;
         focusChan = -1;
      }
   }
   else
   {
      focusActive = false;
      focusChan = -1;
      focusHoldCyclesRemaining = 0;
      lastRequestedMux = -1; // allow fresh mux requests after leaving focus
   }

   // Manage periodic safety sweep while in focus mode
   if (focusActive)
   {
      if (!safetySweepActive)
      {
         if (safetyCountdownCycles > 0) safetyCountdownCycles--;
         if (safetyCountdownCycles <= 0)
         {
            // Start a safety sweep: disable balancing and sweep all channels once
            safetySweepActive = true;
            safetyReadCount = 0;
            // Ensure sweep starts at channel 0 for canonical coverage
            chan = 0;
            safetyPrimed = false; // we must prime ADC for channel 0 before using the result
            lastRequestedMux = -1; // reset to allow prime mux request
            // On old HW (<=2.2) give the mux extra time to settle: skip one extra 25ms cycle
            int hwrev = Param::GetInt(Param::hwrev);
            safetyPrimeWaitCycles = (hwrev <= 4 /* HW_22 or older */) ? 2 : 1;
            // Clear any lingering per-cell command indicators; during sweep all balancing is off
            int numChanClr = Param::GetInt(Param::numchan);
            for (int i = 0; i < numChanClr; i++)
            {
               Param::SetInt((Param::PARAM_NUM)(Param::u0cmd + i), (int)FlyingAdcBms::BAL_OFF);
            }
            // Reset countdown now for the next interval, it will continue ticking after sweep
            safetyCountdownCycles = (safetyIntervalSec * 1000) / 25;
         }
      }
   }
   else
   {
      // Not in focus mode: no special sweeping
      safetySweepActive = false;
      safetyReadCount = 0;
      safetyCountdownCycles = (safetyIntervalSec * 1000) / 25;
   }

   bool inIdle = Param::GetInt(Param::opmode) == BmsFsm::IDLE;
   bool ubalanceOk = Param::GetFloat(Param::uavg) > Param::GetFloat(Param::ubalance);
   bool balance = inIdle && (BAL_OFF != balMode) && (ubalanceOk || focusActive);
   FlyingAdcBms::BalanceStatus bstt;

   // During a safety sweep we force balancing off to get fast, clean readings
   if (safetySweepActive)
   {
      balance = false;
   }

   if (balance)
   {
      // If we are dwelling on focus and not in a safety sweep, continuously charge the focus cell
      if (focusActive && !safetySweepActive && focusChan >= 0)
      {
         chan = (uint8_t)focusChan;
         // Ensure no other cell shows a command during dwell
         int numChanNow = Param::GetInt(Param::numchan);
         for (int i = 0; i < numChanNow; i++)
         {
            if (i == chan) continue;
            Param::SetInt((Param::PARAM_NUM)(Param::u0cmd + i), (int)FlyingAdcBms::BAL_OFF);
         }

         if (muxSettledChannel != (int)chan)
         {
            muxRequest = chan;
            lastRequestedMux = chan;
            FlyingAdcBms::BalanceStatus offStt = FlyingAdcBms::SetBalancing(FlyingAdcBms::BAL_OFF);
            Param::SetInt((Param::PARAM_NUM)(Param::u0cmd + chan), offStt);
            balanceCycles = 1; // keep read block from running while focus mux is settling
            return;
         }

         int effBalMode = balMode & ~BAL_DIS; // never discharge during focus
         FlyingAdcBms::BalanceStatus bsttLocal;
         if (effBalMode & BAL_ADD)
         {
            bsttLocal = FlyingAdcBms::SetBalancing(FlyingAdcBms::BAL_CHARGE);
         }
         else
         {
            bsttLocal = FlyingAdcBms::SetBalancing(FlyingAdcBms::BAL_OFF);
         }
         Param::SetInt((Param::PARAM_NUM)(Param::u0cmd + chan), bsttLocal);
         // Skip cycle-based timing here; metrics update on safety sweeps only
         balanceCycles = 1; // keep read block from running during dwell
      }
      else
      {
      if (balanceCycles == 0)
      {
         balanceCycles = totalBalanceCycles; // this leads to switching to next channel below
      }
      else
      {
         balanceCycles--;
      }

      if (balanceCycles > 0 && balanceCycles < (totalBalanceCycles - 1))
      {
         // During focus dwell ensure we are balancing the focus channel
         if (focusActive && focusChan >= 0)
         {
            chan = (uint8_t)focusChan;
            if (lastRequestedMux != (int)chan) { muxRequest = chan; lastRequestedMux = chan; } // keep hardware on focus channel during balancing window
         }
         float udc = Param::GetFloat((Param::PARAM_NUM)(Param::u0 + chan));
         float balanceTarget = 0;
         int numChan = Param::GetInt(Param::numchan);
         // Use trimmed average excluding lowest cell when BOTH is selected
         switch (balMode)
         {
            case BAL_ADD:
               balanceTarget = Param::GetFloat(Param::umax);
               break;
            case BAL_DIS:
               balanceTarget = Param::GetFloat(Param::umin);
               break;
            case BAL_BOTH:
            default:
               balanceTarget = ComputeTrimmedAvgExcludingMin(numChan);
               break;
         }

         // Only disallow discharge while focusing a weak cell; otherwise allow standard behavior
         int effBalMode = focusActive ? (balMode & ~BAL_DIS) : balMode;

         if (focusActive && chan == focusChan && (effBalMode & BAL_ADD))
         {
            // Focus condition already vetted against balFocusMinDev during selection.
            // Avoid missing charge due to small measurement noise; charge while focused.
            bstt = FlyingAdcBms::SetBalancing(FlyingAdcBms::BAL_CHARGE);
         }
         else if (udc < (balanceTarget - 3) && (effBalMode & BAL_ADD))
         {
            bstt = FlyingAdcBms::SetBalancing(FlyingAdcBms::BAL_CHARGE);
         }
         else if (udc > (balanceTarget + 1) && (effBalMode & BAL_DIS))
         {
            bstt = FlyingAdcBms::SetBalancing(FlyingAdcBms::BAL_DISCHARGE);
         }
         else
         {
            bstt = FlyingAdcBms::SetBalancing(FlyingAdcBms::BAL_OFF);
            balanceCycles = 0;
         }
         Param::SetInt((Param::PARAM_NUM)(Param::u0cmd + chan), bstt);
      }
      else
      {
         FlyingAdcBms::SetBalancing(FlyingAdcBms::BAL_OFF);
      }
      }
   }
   else
   {
      balanceCycles = totalBalanceCycles;
      bstt = FlyingAdcBms::SetBalancing(FlyingAdcBms::BAL_OFF);
      Param::SetInt((Param::PARAM_NUM)(Param::u0cmd + chan), bstt);
   }

   // Read cell voltage when balancing is turned off (mainline behavior)
   if (balanceCycles == totalBalanceCycles)
   {
      float gain = Param::GetFloat(Param::gain);
      int numChan = Param::GetInt(Param::numchan);
      bool even = (chan & 1) == 0;

      if (chan == 0)
         gain *= 1 + Param::GetFloat(Param::correction0) / 1000000.0f;
      else if (chan == 1)
         gain *= 1 + Param::GetFloat(Param::correction1) / 1000000.0f;
      else if (chan == 15)
         gain *= 1 + Param::GetFloat(Param::correction15) / 1000000.0f;

      //Read ADC result before mux change
      float udc = FlyingAdcBms::GetResult() * (gain / 1000.0f);

      // If we just entered a safety sweep, the first ADC result still belongs to the previous
      // channel (from focus dwell). Prime by switching mux to chan=0 and skip storing/aggregation
      // this cycle so the next 25ms result maps to u0 correctly.
      if (safetySweepActive && safetyReadCount == 0 && safetyPrimeWaitCycles > 0)
      {
         // First prime cycle: request channel 0 once, then wait required cycles
         if (!safetyPrimed)
         {
            muxRequest = chan; // chan is 0 at sweep start
            safetyPrimed = true;
            lastRequestedMux = chan;
         }
         safetyPrimeWaitCycles--;
         return;
      }

      Param::SetFloat((Param::PARAM_NUM)(Param::u0 + chan), udc);

      if (safetySweepActive)
      {
         // At the very start of a safety sweep, reset aggregator
         if (safetyReadCount == 0)
         {
            sum = 0;
            min = 8000;
            max = 0;
         }

         // Aggregate this channel reading into sum/min/max only during safety sweep
         min = MIN(min, udc);
         max = MAX(max, udc);
         sum += udc;

         // Progress normally through a full sweep with balancing disabled
         if (even && (chan + 2) < numChan)
            chan += 2;
         else if (even && (chan + 1) < numChan)
            chan++;
         else if (even)
            chan--;
         else if (chan > 1)
            chan -= 2;
         else
         {
            chan = 0;
         }
         // Count reads to know when a full sweep is done
         safetyReadCount++;
         if (safetyReadCount >= numChan)
         {
            // Completed a full safety sweep: now publish aggregated metrics
            Accumulate(sum, min, max, (numChan > 0 ? (sum / numChan) : 0));
            min = 8000;
            max = 0;
            sum = 0;
            safetySweepActive = false;
            safetyReadCount = 0;
            // Fresh data now visible via stored u-values; normal selection logic will use them
         }
         muxRequest = chan;
      }
      else if (focusActive)
      {
         // Freeze on focus channel for dwell balancing; keep mux on focus channel
         chan = (focusChan >= 0 && focusChan < numChan) ? (uint8_t)focusChan : 0;
         if (lastRequestedMux != (int)chan) { muxRequest = chan; lastRequestedMux = chan; }
         // Do NOT modify sum/min/max while frozen; aggregation happens in safety sweeps
      }
      else
      {
         // Normal scanning (focus inactive): aggregate and Accumulate at wrap
         min = MIN(min, udc);
         max = MAX(max, udc);
         sum += udc;

         // Normal channel progression
         if (even && (chan + 2) < numChan)
            chan += 2;
         else if (even && (chan + 1) < numChan)
            chan++;
         else if (even)
            chan--;
         else if (chan > 1)
            chan -= 2;
         else
         {
            chan = 0;
            Accumulate(sum, min, max, sum / numChan);
            min = 8000;
            max = 0;
            sum = 0;
         }
         muxRequest = chan;
      }
   }
}

void BmsIO::ReadTemperatures()
{
   int sensor = Param::GetInt(Param::tempsns);
   int nomRes = Param::GetInt(Param::tempres);
   int beta = Param::GetInt(Param::tempbeta);
   float temp1 = NO_TEMP, temp2 = NO_TEMP, tempmin = NO_TEMP, tempmax = NO_TEMP;

   if (sensor & 1)
      tempmin = tempmax = temp1 = TempMeas::AdcToTemperature(AnaIn::temp1.Get(), nomRes, beta);

   if (sensor & 2)
      tempmin = tempmax = temp2 = TempMeas::AdcToTemperature(AnaIn::temp2.Get(), nomRes, beta);

   if (sensor == 3) //two sensors, calculate min and max
   {
      tempmin = MIN(temp1, temp2);
      tempmax = MAX(temp1, temp2);
   }

   Param::SetFloat(Param::tempmin0, tempmin);
   Param::SetFloat(Param::tempmax0, tempmax);
}

void BmsIO::MeasureCurrent()
{
   int idcmode = Param::GetInt(Param::idcmode);

   if (idcmode == IDC_DIFFERENTIAL || idcmode == IDC_SINGLE)
   {
      float current = 0;
      static int samples = 0;
      static u32fp amsIn = 0, amsOut = 0;
      static float idcavg = 0;
      int curpos = AnaIn::curpos.Get();
      int curneg = AnaIn::curneg.Get();
      float idcgain = Param::GetFloat(Param::idcgain);
      int idcofs = Param::GetInt(Param::idcofs);
      int rawCurrent = idcmode == IDC_SINGLE ? curpos : curpos - curneg;

      current = (rawCurrent - idcofs) / idcgain;

      if (current < -0.8f)
      {
         amsOut += -FP_FROMFLT(current);
      }
      else if (current > 0.8f)
      {
         amsIn += FP_FROMFLT(current);
      }

      idcavg += current;
      samples++;

      if (samples == 200)
      {
         s32fp chargein = Param::Get(Param::chargein);
         s32fp chargeout = Param::Get(Param::chargeout);

         chargein += amsIn / 200;
         chargeout += amsOut / 200;
         idcavg /= 200;

         float voltage = Param::GetFloat(Param::utotal) / 1000;
         float power = voltage * idcavg;

         Param::SetFloat(Param::idcavg, idcavg);
         Param::SetFloat(Param::power, power);

         amsIn = 0;
         amsOut = 0;
         samples = 0;
         idcavg = 0;

         //BmsCalculation::SetCharge(chargein, chargeout);
         Param::SetFixed(Param::chargein, chargein);
         Param::SetFixed(Param::chargeout, chargeout);
      }
      Param::SetFloat(Param::idc, current);
   }
}

void BmsIO::TestReadCellVoltage(int chan, FlyingAdcBms::BalanceCommand cmd)
{
   float gain = Param::GetFloat(Param::gain);

   if (chan == 0)
      gain *= 1 + Param::GetFloat(Param::correction0) / 1000000.0f;
   else if (chan == 1)
      gain *= 1 + Param::GetFloat(Param::correction1) / 1000000.0f;
   else if (chan == 15)
      gain *= 1 + Param::GetFloat(Param::correction15) / 1000000.0f;

   float udc = FlyingAdcBms::GetResult() * (gain / 1000.0f);;
   FlyingAdcBms::SelectChannel(chan);
   FlyingAdcBms::SetBalancing(cmd);
   FlyingAdcBms::StartAdc();
   muxSettledChannel = chan;
   Param::SetFloat((Param::PARAM_NUM)(Param::u0 + chan), udc);
}


void BmsIO::Accumulate(float sum, float min, float max, float avg)
{

   if (bmsFsm->IsFirst())
   {
      // For master node
      Param::SetFloat(Param::uavg0, avg);
      Param::SetFloat(Param::umin0, min);
      Param::SetFloat(Param::umax0, max);

      float totalSum = sum, totalMin = min, totalMax = max;
      //If we are the first module accumulate our values with those from the sub modules
      for (int i = 1; i < bmsFsm->GetNumberOfModules(); i++)
      {
         //Here we undo the local average calculation on the module to calculate the substrings total voltage
         float subAvg = Param::GetFloat(bmsFsm->GetDataItem(Param::uavg0, i));
         float subMin = Param::GetFloat(bmsFsm->GetDataItem(Param::umin0, i));
         float subMax = Param::GetFloat(bmsFsm->GetDataItem(Param::umax0, i));
         
         totalSum += subAvg * bmsFsm->GetCellsOfModule(i);
         totalMin = MIN(totalMin, subMin);
         totalMax = MAX(totalMax, subMax);
      }

      float tempmin = NO_TEMP, tempmax = -40;

      for (int i = 0; i < bmsFsm->GetNumberOfModules(); i++)
      {
         float tempmin0 = Param::GetFloat(bmsFsm->GetDataItem(Param::tempmin0, i));
         float tempmax0 = Param::GetFloat(bmsFsm->GetDataItem(Param::tempmax0, i));

         if (tempmin0 < NO_TEMP)
         {
            tempmin = MIN(tempmin, tempmin0);
            tempmax = MAX(tempmax, tempmax0);
         }
      }

      Param::SetFloat(Param::umin, totalMin);
      Param::SetFloat(Param::umax, totalMax);
      Param::SetFloat(Param::uavg, totalSum / Param::GetInt(Param::totalcells));
      Param::SetFloat(Param::udelta, totalMax - totalMin);
      Param::SetFloat(Param::utotal, totalSum);
      Param::SetFloat(Param::tempmin, tempmin);
      Param::SetFloat(Param::tempmax, tempmax);
      
      // Update VX1-specific parameters for display and external use
      Param::SetFloat(Param::VX1umin, totalMin);
      Param::SetFloat(Param::VX1umax, totalMax);
      Param::SetFloat(Param::VX1uavg, totalSum / Param::GetInt(Param::totalcells));
      Param::SetFloat(Param::VX1udelta, totalMax - totalMin);
      Param::SetFloat(Param::VX1utotal, totalSum);
      Param::SetFloat(Param::VX1tempmin, tempmin);
      Param::SetFloat(Param::VX1tempmax, tempmax);
   }
   else //if we are a sub module
   {
      // For sub-modules
      
      // Set local values that we measured directly
      Param::SetFloat(Param::utotal, sum);
      Param::SetFloat(Param::uavg0, avg);
      Param::SetFloat(Param::umin0, min);
      Param::SetFloat(Param::umax0, max);
      Param::SetFloat(Param::udelta, max - min);
      
      // Update VX1-specific parameters
      Param::SetFloat(Param::VX1utotal, sum);
      
      // Get master values
      float umin = Param::GetFloat(Param::umin);
      float umax = Param::GetFloat(Param::umax);
      float uavg = Param::GetFloat(Param::uavg);
      
      // Set VX1 values
      Param::SetFloat(Param::VX1uavg, uavg);
      Param::SetFloat(Param::VX1umin, umin);
      Param::SetFloat(Param::VX1umax, umax);
      Param::SetFloat(Param::VX1udelta, umax - umin);
   }
}
