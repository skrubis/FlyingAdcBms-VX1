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

void BmsIO::ReadCellVoltages()
{
   const int totalBalanceCycles = 30;
   static uint16_t balanceCycles[18] = {0};
   static uint8_t chan = 0;
   static float sum = 0, min, max, avg;
   int balMode = Param::GetInt(Param::balmode);
   bool balance = Param::GetInt(Param::opmode) == BmsFsm::IDLE && Param::GetFloat(Param::uavg) > Param::GetFloat(Param::ubalance) && BAL_OFF != balMode;
   FlyingAdcBms::BalanceStatus bstt;

   if (balance)
   {
      if (balanceCycles[chan] == 0)
      {
         balanceCycles[chan] = totalBalanceCycles; //this leads to switching to next channel below
      }
      else
      {
         balanceCycles[chan]--;
      }

      if (balanceCycles[chan] > 0 && balanceCycles[chan] < (totalBalanceCycles - 1))
      {
         float udc = Param::GetFloat((Param::PARAM_NUM)(Param::u0 + chan));
         float balanceTarget = 0;

         int balanceMode = Param::GetInt(Param::balmode);
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

         // Calculate adaptive thresholds based on voltage delta and minimum values
         float deltaV = Param::GetFloat(Param::umax) - Param::GetFloat(Param::umin);
         float chargeThreshold = MAX(3.0f, deltaV * 0.02f);  // At least 3mV or 2% of delta
         float dischargeThreshold = MAX(1.0f, deltaV * 0.01f);  // At least 1mV or 1% of delta
         
         // Static variables to cache global values for slave nodes
         static float cachedUmin = 0.0f;
         static float cachedUmax = 0.0f;
         static float cachedUavg = 0.0f;
         static uint32_t lastCacheUpdateTime = 0;
         static uint32_t cacheCounter = 0;
         cacheCounter++;
         
         // Cache values with a timeout mechanism for slave nodes
         if (!bmsFsm->IsFirst()) {
             // Current cached values
             float currentUmin = Param::GetFloat(Param::umin);
             float currentUmax = Param::GetFloat(Param::umax);
             float currentUavg = Param::GetFloat(Param::uavg);
             
             // Check if values changed significantly (received update from master)
             bool valuesChanged = (ABS(currentUmin - cachedUmin) > 1.0f) || 
                                 (ABS(currentUmax - cachedUmax) > 1.0f) || 
                                 (ABS(currentUavg - cachedUavg) > 1.0f);
             
             if (valuesChanged) {
                 // Update cached values
                 cachedUmin = currentUmin;
                 cachedUmax = currentUmax;
                 cachedUavg = currentUavg;
                 lastCacheUpdateTime = cacheCounter;
             }
             
             // Make balancing decisions more sensitive on slave nodes
             chargeThreshold = MAX(1.5f, deltaV * 0.01f);    // More aggressive (1.5mV minimum)
             dischargeThreshold = MAX(0.5f, deltaV * 0.005f); // More aggressive (0.5mV minimum)
             
             // If cache is stale (no updates for 200 cycles ~5 seconds), use even more aggressive thresholds
             if ((cacheCounter - lastCacheUpdateTime) > 200) {
                 // Use more aggressive thresholds if not receiving regular updates
                 chargeThreshold = MAX(1.0f, deltaV * 0.005f);     // Very aggressive
                 dischargeThreshold = MAX(0.3f, deltaV * 0.0025f); // Very aggressive
             }
             
             // Special case for significant outliers
             float significantDeviation = 50.0f; // mV
             if (ABS(udc - balanceTarget) > significantDeviation) {
                 // For big deviations, use even more aggressive thresholds
                 if (udc < balanceTarget)
                     chargeThreshold = 1.0f; // Extremely aggressive charge threshold for low cells
                 else
                     dischargeThreshold = 0.3f; // Extremely aggressive discharge threshold for high cells
             }
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
            balanceCycles[chan] = 0;
         }
         Param::SetInt((Param::PARAM_NUM)(Param::u0cmd + chan), bstt);
      }
      else
      {
         FlyingAdcBms::SetBalancing(FlyingAdcBms::BAL_OFF);
      }
   }
   else
   {
      balanceCycles[chan] = totalBalanceCycles;
      bstt = FlyingAdcBms::SetBalancing(FlyingAdcBms::BAL_OFF);
      Param::SetInt((Param::PARAM_NUM)(Param::u0cmd + chan), bstt);
   }

   //Read cell voltage when balancing is turned off
   if (balanceCycles[chan] >= totalBalanceCycles)
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
      //We have no reached chan 1. Accumulate values and restart at chan 0
      else
      {
         chan = 0;
         avg = sum / Param::GetInt(Param::numchan);
         Accumulate(sum, min, max, avg);

         min = 8000;
         max = 0;
         sum = 0;
      }

      FlyingAdcBms::SelectChannel(chan);
      FlyingAdcBms::StartAdc();
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

// CorrectVoltage function removed - no longer needed with CAN_SIGNED=1

// CorrectAllVoltages function removed - no longer needed with CAN_SIGNED=1

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
      
      // Update VX1-specific parameters (from previous fix)
      // This ensures VX1 uses the corrected values without modifying BMS calculations
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
      // For sub-modules - also apply correction to values received from master/other nodes
      
      // Set local values that we measured directly
      Param::SetFloat(Param::utotal, sum);
      Param::SetFloat(Param::uavg0, avg);
      Param::SetFloat(Param::umin0, min);
      Param::SetFloat(Param::umax0, max);
      Param::SetFloat(Param::udelta, max - min);
      
      // With CAN_SIGNED=1, values from CAN communication are correctly interpreted without correction
      Param::SetFloat(Param::uavg, Param::GetFloat(Param::uavg));
      Param::SetFloat(Param::umin, Param::GetFloat(Param::umin));
      Param::SetFloat(Param::umax, Param::GetFloat(Param::umax));
      
      // Update VX1-specific parameters (no correction needed with CAN_SIGNED=1)
      Param::SetFloat(Param::VX1utotal, sum);
      Param::SetFloat(Param::VX1uavg, Param::GetFloat(Param::uavg));
      Param::SetFloat(Param::VX1umin, Param::GetFloat(Param::umin));
      Param::SetFloat(Param::VX1umax, Param::GetFloat(Param::umax));
      Param::SetFloat(Param::VX1udelta, Param::GetFloat(Param::umax) - Param::GetFloat(Param::umin));
   }
}
