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

#ifndef VX1_H
#define VX1_H

#include "params.h"
#include "canhardware.h"

/**
 * @brief VX1 module for handling VX1-specific functionality
 * 
 * This class provides methods for managing VX1 mode and related features.
 */
class VX1
{
public:
    /**
     * @brief Initialize VX1 module
     */
    static void Initialize();
    
    /**
     * @brief Get the current VX1 mode
     * 
     * @return true if VX1 mode is enabled, false otherwise
     */
    static bool IsEnabled();
    
    /**
     * @brief Get the appropriate CAN baud rate based on VX1 mode
     * 
     * @return CanHardware::baudrates The baud rate to use (Baud250 if VX1 enabled, Baud500 otherwise)
     */
    static CanHardware::baudrates GetCanBaudRate();
    
    /**
     * @brief Handle parameter changes related to VX1
     * 
     * @param paramNum The parameter that changed
     */
    static void HandleParamChange(Param::PARAM_NUM paramNum);
};

#endif // VX1_H
