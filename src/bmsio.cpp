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
#include <cmath> // For fabs function

BmsFsm* BmsIO::bmsFsm;
int BmsIO::muxRequest = -1;
uint8_t BmsIO::cellsNeedingBalance[16] = {0};
int BmsIO::totalCellsNeedingBalance = 0;

// Check if battery is at high voltage (above safe threshold)
bool BmsIO::IsAboveSafeVoltage()
{
    float safeVolt = Param::GetFloat(Param::safeVoltage);
    return Param::GetFloat(Param::umax) > safeVolt;
}

// Scale the extra balancing cycles based on voltage and cell requirements
int BmsIO::ScaleExtraBalancingCycles(int baseCycles)
{
    int maxDelayMs = Param::GetInt(Param::maxScanDelay) * 1000;
    int cycleTimeMs = 25; // 25ms per cycle
    int maxTotalCycles = maxDelayMs / cycleTimeMs;
    float reductionFactor = 1.0f;
    
    // Reduce by factor of 0.3 if above safe voltage 
    if (IsAboveSafeVoltage()) {
        reductionFactor = 0.3f;
    }
    
    // If we have multiple cells needing balancing, reduce cycles to stay under total delay
    if (totalCellsNeedingBalance > 1) {
        int maxCyclesPerCell = maxTotalCycles / totalCellsNeedingBalance;
        // Ensure we don't exceed the overall limit
        reductionFactor = MIN(reductionFactor, (float)maxCyclesPerCell / baseCycles);
    }
    
    return (int)(baseCycles * reductionFactor);
}

// Identify which cells need extended balancing
void BmsIO::UpdateCellBalancingNeeds()
{
    totalCellsNeedingBalance = 0;
    float balanceTarget = 0;
    int balMode = Param::GetInt(Param::balmode);
    int numChan = Param::GetInt(Param::numchan);
    
    // Skip if balancing is off
    if (balMode == BAL_OFF) {
        return;
    }
    
    // Determine target voltage based on balance mode
    switch (balMode)
    {
        case BAL_ADD: // Maximum cell voltage is target when only adding
            balanceTarget = Param::GetFloat(Param::umax);
            break;
        case BAL_DIS: // Minimum cell voltage is target when only dissipating
            balanceTarget = Param::GetFloat(Param::umin);
            break;
        case BAL_BOTH: // Average cell voltage is target when dissipating and adding
            balanceTarget = Param::GetFloat(Param::uavg);
            break;
        default: 
            return;
    }
    
    // Check each cell against target
    for (int i = 0; i < numChan; i++) {
        float udc = Param::GetFloat((Param::PARAM_NUM)(Param::u0 + i));
        float deviation = fabs(udc - balanceTarget);
        
        if (deviation > 15.0f) {
            cellsNeedingBalance[i] = 1;
            totalCellsNeedingBalance++;
        } else {
            cellsNeedingBalance[i] = 0;
        }
    }
}

/** \brief Mux control function. Must be called in 2 ms interval */
void BmsIO::SwitchMux()
{
   static int channel = -1;
   static bool startAdc = false;

   //t=0 ms: On a mux change request first completely turn off mux
   if (muxRequest >= 0)
   {
      FlyingAdcBms::MuxOff();
      channel = muxRequest;
      muxRequest = -1;
   }
   //t=2 ms: switch to requested channel
   else if (channel >= 0)
   {
      FlyingAdcBms::SelectChannel(channel);
      channel = -1;
      startAdc = true;
   }
   //t=4 ms: start ADC
   else if (startAdc)
   {
      FlyingAdcBms::StartAdc();
      startAdc = false;
   }
   //t=21 ms: ADC conversion is finished
   //t=25 ms: ADC conversion result is read
}

void BmsIO::ReadCellVoltages()
{
   const int totalBalanceCycles = 30;
   static uint8_t chan = 0, balanceCycles = 0;
   static int extraBalanceCycles = 0;
   static float sum = 0, min = 8000, max = 0;
   int balMode = Param::GetInt(Param::balmode);
   bool balance = Param::GetInt(Param::opmode) == BmsFsm::IDLE && Param::GetFloat(Param::uavg) > Param::GetFloat(Param::ubalance) && BAL_OFF != balMode;
   FlyingAdcBms::BalanceStatus bstt;
   
   // Update which cells need balancing (once per full cycle)
   if (chan == 0 && balanceCycles == 0 && extraBalanceCycles == 0) {
      UpdateCellBalancingNeeds();
   }

   if (balance)
   {
      if (balanceCycles == 0 && extraBalanceCycles == 0)
      {
         balanceCycles = totalBalanceCycles; // This leads to switching to next channel below
      }
      else if (extraBalanceCycles > 0)
      {
         extraBalanceCycles--;
      }
      else if (balanceCycles > 0)
      {
         balanceCycles--;
      }

      if ((balanceCycles > 0 && balanceCycles < (totalBalanceCycles - 1)) || extraBalanceCycles > 0)
      {
         float udc = Param::GetFloat((Param::PARAM_NUM)(Param::u0 + chan));
         float balanceTarget = 0;

         switch (balMode)
         {
         case BAL_ADD: //maximum cell voltage is target when only adding
            balanceTarget = Param::GetFloat(Param::umax);
            break;
         case BAL_DIS: //minimum cell voltage is target when only dissipating
            balanceTarget = Param::GetFloat(Param::umin);
            break;
         case BAL_BOTH: //average cell voltage is target when dissipating and adding
            balanceTarget = Param::GetFloat(Param::uavg);
            break;
         default: //not balancing
            break;
         }
         
         // Calculate the deviation to determine aggressiveness
         float deviation = fabs(udc - balanceTarget);
         float chargeThreshold = 3.0f;
         float dischargeThreshold = 1.0f;
         
         // More aggressive thresholds for cells with larger deviations
         if (deviation > 15.0f) {
            // Use more aggressive thresholds for cells that need balancing badly
            chargeThreshold = 1.5f;
            dischargeThreshold = 0.5f;
         }

         if (udc < (balanceTarget - chargeThreshold) && (balMode & BAL_ADD))
         {
            bstt = FlyingAdcBms::SetBalancing(FlyingAdcBms::BAL_CHARGE);
         }
         else if (udc > (balanceTarget + dischargeThreshold) && (balMode & BAL_DIS))
         {
            bstt = FlyingAdcBms::SetBalancing(FlyingAdcBms::BAL_DISCHARGE);
         }
         else
         {
            bstt = FlyingAdcBms::SetBalancing(FlyingAdcBms::BAL_OFF);
            if (extraBalanceCycles > 0) {
               // Keep track of extra balancing cycles separately
               extraBalanceCycles = 0;
            } else {
               balanceCycles = 0;
            }
         }
         Param::SetInt((Param::PARAM_NUM)(Param::u0cmd + chan), bstt);
      }
      else if (balanceCycles == 1) // When we're about to finish balancing
      {
         // When we're about to finish the normal balancing cycle, check if this cell needs extended balancing
         float udc = Param::GetFloat((Param::PARAM_NUM)(Param::u0 + chan));
         float balanceTarget = 0;
         
         // Get the appropriate balance target based on mode (same code as above)
         switch (balMode)
         {
         case BAL_ADD:
            balanceTarget = Param::GetFloat(Param::umax);
            break;
         case BAL_DIS:
            balanceTarget = Param::GetFloat(Param::umin);
            break;
         case BAL_BOTH:
            balanceTarget = Param::GetFloat(Param::uavg);
            break;
         default:
            break;
         }
         
         float deviation = fabs(udc - balanceTarget);
         
         // Determine if cell needs extended balancing and calculate extra cycles
         if (deviation > 30.0f && cellsNeedingBalance[chan]) {
            int extraCycles = Param::GetInt(Param::balDeltaHigh);
            extraBalanceCycles = ScaleExtraBalancingCycles(extraCycles);
         } 
         else if (deviation > 15.0f && cellsNeedingBalance[chan]) {
            int extraCycles = Param::GetInt(Param::balDeltaMed);
            extraBalanceCycles = ScaleExtraBalancingCycles(extraCycles);
         }
         
         FlyingAdcBms::SetBalancing(FlyingAdcBms::BAL_OFF);
      }
      else
      {
         FlyingAdcBms::SetBalancing(FlyingAdcBms::BAL_OFF);
      }
   }
   else
   {
      balanceCycles = totalBalanceCycles;
      bstt = FlyingAdcBms::SetBalancing(FlyingAdcBms::BAL_OFF);
      Param::SetInt((Param::PARAM_NUM)(Param::u0cmd + chan), bstt);
   }

   // Read cell voltage when balancing is turned off AND no extra balancing cycles remain
   if (balanceCycles == totalBalanceCycles && extraBalanceCycles == 0)
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

      Param::SetFloat((Param::PARAM_NUM)(Param::u0 + chan), udc);

      min = MIN(min, udc);
      max = MAX(max, udc);
      sum += udc;

      //First we sweep across all even channels: 0, 2, 4,...
      if (even && (chan + 2) < numChan)
         chan += 2;
      //After reaching the furthest even channel (say 12) we either change over to a higher odd channel
      else if (even && (chan + 1) < numChan)
         chan++;
      //or lower odd channel
      else if (even)
         chan--;
      //Now we sweep across all odd channels until we reach 1
      else if (chan > 1)
         chan -= 2;
      //We have now reached chan 1. Accumulate values and restart at chan 0
      else
      {
         chan = 0;
         Accumulate(sum, min, max, sum / Param::GetInt(Param::numchan));

         min = 8000;
         max = 0;
         sum = 0;
      }

      //This instructs the SwitchMux task to change channel, with dead time
      muxRequest = chan;
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
