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

// External reference to the BmsFsm instance from main.cpp
extern BmsFsm* bmsFsm;

// Define static class members
char VX1::odometerMessage[7] = "      "; // Initialize with spaces
bool VX1::displayActive = false;

// J1939 PGN for VX1 odometer display
#define VX1_ODOMETER_PGN 0x00FEED
#define VX1_OVERRIDE_NORMAL 0x55
#define VX1_OVERRIDE_FORCE 0xAA

/**
 * Initialize VX1 module
 */
void VX1::Initialize()
{
    // Any initialization code can go here
    displayActive = false;
    memset(odometerMessage, ' ', 6); // Initialize with spaces
    odometerMessage[6] = '\0'; // Null terminator
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
 * Check if this node is the master node
 * 
 * @param bmsFsm Pointer to the BmsFsm instance
 * @return true if this is the master node (first node with ID 10 by default)
 */
bool VX1::IsMaster(BmsFsm* bmsFsmInstance)
{
    // Use the provided BmsFsm instance or the global one if not provided
    BmsFsm* fsm = bmsFsmInstance ? bmsFsmInstance : bmsFsm;
    
    // If we have a valid BmsFsm instance, use its IsFirst method
    if (fsm != nullptr)
    {
        return fsm->IsFirst();
    }
    
    // Fallback: check if module address is 10 (default master node ID)
    return Param::GetInt(Param::modaddr) == 10;
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

/**
 * Set the message to be displayed on the odometer
 * 
 * @param message The message to display (max 6 characters)
 */
void VX1::SetOdometerMessage(const char* message)
{
    // Copy up to 6 characters, pad with spaces if needed
    int len = strlen(message);
    if (len > 6) len = 6;
    
    // Initialize with spaces
    memset(odometerMessage, ' ', 6);
    
    // Copy the message
    memcpy(odometerMessage, message, len);
    odometerMessage[6] = '\0'; // Ensure null termination
    
    displayActive = true;
}

/**
 * Send a message to the VX1 odometer display
 * 
 * @param message The message to display (max 6 characters)
 * @param canHardware Pointer to the CAN hardware interface
 * @param sourceAddress Source address for the J1939 message (default 0x80)
 * @return true if message was sent successfully
 */
bool VX1::SendOdometerMessage(const char* message, CanHardware* canHardware, uint8_t sourceAddress, bool masterOnly)
{
    // Check if VX1 mode is enabled and we have a valid CAN interface
    if (!IsEnabled() || !canHardware)
        return false;
        
    // If masterOnly is true, check if this is the master node
    if (masterOnly && !IsMaster())
        return false;
        
    // Set the message if provided
    if (message)
    {
        SetOdometerMessage(message);
    }
    
    // Prepare the J1939 message
    uint32_t data[2] = {0, 0};
    uint8_t* bytes = (uint8_t*)data;
    
    // Characters are in reverse order (byte 5 is leftmost, byte 0 is rightmost)
    bytes[0] = odometerMessage[5]; // Right-most character
    bytes[1] = odometerMessage[4];
    bytes[2] = odometerMessage[3];
    bytes[3] = odometerMessage[2];
    bytes[4] = odometerMessage[1];
    bytes[5] = odometerMessage[0]; // Left-most character
    bytes[6] = 0x00;               // Charger active indicator (off)
    bytes[7] = VX1_OVERRIDE_FORCE; // Override control (force display)
    
    // Calculate the J1939 29-bit ID
    // Format: Priority (3 bits) | PGN (18 bits) | Source Address (8 bits)
    // Priority 3 (0b011) << 26 | PGN 0x00FEED << 8 | Source Address
    uint32_t j1939Id = (3 << 26) | (VX1_ODOMETER_PGN << 8) | sourceAddress;
    
    // Send the message
    canHardware->Send(j1939Id, data, 8);
    
    return true;
}

/**
 * Task to periodically send the odometer message (call every 100ms)
 * 
 * This should be added to the scheduler if continuous display is needed
 * @param canHardware Pointer to the CAN hardware interface
 */
void VX1::OdometerDisplayTask(CanHardware* canHardware, bool masterOnly)
{
    // Only proceed if display is active and VX1 mode is enabled
    // If masterOnly is true, also check if this is the master node
    if (displayActive && IsEnabled() && (!masterOnly || IsMaster()))
    {
        SendOdometerMessage(nullptr, canHardware, 0x80, masterOnly);
    }
}
