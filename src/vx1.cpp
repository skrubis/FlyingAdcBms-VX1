/*
 * This file is part of the FlyingAdcBms-VX1 project.
 *
 * Copyright (C) 2025 FlyingAdcBms Team
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

#include "vx1.h"
// param_prj.h is already included via params.h in vx1.h

/**
 * Initialize VX1 module
 */
void VX1::Initialize()
{
    // Any initialization code can go here
    // Currently, no specific initialization is needed
}

/**
 * Check if VX1 mode is enabled
 * 
 * @return true if VX1 mode is enabled, false otherwise
 */
bool VX1::IsEnabled()
{
    return Param::GetInt(Param::vx1mode) == 1;
}

/**
 * Get the appropriate CAN baud rate based on VX1 mode
 * 
 * @return CanHardware::baudrates The baud rate to use (Baud250 if VX1 enabled, Baud500 otherwise)
 */
CanHardware::baudrates VX1::GetCanBaudRate()
{
    return IsEnabled() ? CanHardware::Baud250 : CanHardware::Baud500;
}

/**
 * Handle parameter changes related to VX1
 * 
 * @param paramNum The parameter that changed
 */
void VX1::HandleParamChange(Param::PARAM_NUM paramNum)
{
    if (paramNum == Param::vx1mode)
    {
        // Handle VX1 mode change
        // Note: CAN baud rate changes will only take effect after restart
    }
}
