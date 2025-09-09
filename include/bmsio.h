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
#ifndef BMSIO_H
#define BMSIO_H

#include "bmsfsm.h"
#include "flyingadcbms.h"

#define NO_TEMP    127


class BmsIO
{
   public:
      static void ReadTemperatures();
      static void SwitchMux();
      static void ReadCellVoltages();
      static void TestReadCellVoltage(int chan, FlyingAdcBms::BalanceCommand cmd);
      static void MeasureCurrent();
      static void SetBmsFsm(BmsFsm* b) { bmsFsm = b; }

   private:
      static void Accumulate(float sum, float min, float max, float avg);
      
      // Helper functions for adaptive balancing
      static bool IsAboveSafeVoltage();
      static int ScaleExtraBalancingCycles(int baseCycles);
      static void UpdateCellBalancingNeeds();
      // Focused weak-cell mode helpers
      static float ComputeBalanceTarget(int balMode);
      static void EvaluateFocusSet();
      
      static BmsFsm* bmsFsm;
      static int muxRequest;
      
      // Tracking for cells needing extended balancing
      static uint8_t cellsNeedingBalance[16]; // Assumes MAX_CELLS=16
      static int totalCellsNeedingBalance;

      // Focus mode state
      static int focusChan;                       // currently focused cell, -1 if none
      static int focusHoldCyclesRemaining;        // remaining 25ms cycles to hold focus
      static bool focusActive;                    // focus mode currently active
      static uint8_t focusSet[16];                // top-N focus mask
      // Periodic refresh during focus
      static bool focusRefreshActive;             // true while we sweep once to refresh globals
      static int focusRefreshCountdownCycles;     // countdown to next refresh (25ms cycles)
};

#endif // BMSIO_H
