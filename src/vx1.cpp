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

// No external references needed

// Define static class members
char VX1::odometerMessage[7] = "      "; // Initialize with spaces
bool VX1::displayActive = false;

// Telltale states
VX1::TelltaleState VX1::wrenchState = VX1::TELLTALE_OFF;
VX1::TelltaleState VX1::batteryState = VX1::TELLTALE_OFF;
VX1::TelltaleState VX1::temperatureState = VX1::TELLTALE_OFF;
VX1::TelltaleState VX1::reservedState = VX1::TELLTALE_OFF;
bool VX1::telltaleActive = false;

// Clock display data
char VX1::clockSegments[5] = "    "; // Initialize with spaces
char VX1::clockChargerIndicator = 0x00;
bool VX1::clockActive = false;

// J1939 PGN for VX1 odometer display
#define VX1_ODOMETER_PGN 0x00FEED
#define VX1_OVERRIDE_NORMAL 0x55
#define VX1_OVERRIDE_FORCE 0xAA

// J1939 PGN for VX1 telltale control
#define VX1_TELLTALE_PGN 0x00FECA

// Boot display sequence states
enum BootDisplayState {
    BOOT_DISPLAY_IDLE,
    BOOT_DISPLAY_WAIT,
    BOOT_DISPLAY_TELLTALE,
    BOOT_DISPLAY_WELCOME,
    BOOT_DISPLAY_UDELTA,
    BOOT_DISPLAY_SOC,
    BOOT_DISPLAY_SOH,
    BOOT_DISPLAY_DONE
};

// Boot display sequence variables
static BootDisplayState bootDisplayState = BOOT_DISPLAY_IDLE;
static uint32_t bootDisplayTimer = 0;
static CanHardware* bootDisplayCanHardware = nullptr;
static uint32_t bootDisplayStartTime = 0;

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
    // If we have a valid BmsFsm instance, use its IsFirst method
    if (bmsFsmInstance != nullptr)
    {
        return bmsFsmInstance->IsFirst();
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
    // Check if VX1 mode is enabled, VX1enCanMsg is set to 1, and we have a valid CAN interface
    if (!IsEnabled() || !canHardware || Param::GetInt(Param::VX1enCanMsg) != 1)
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

/**
 * Set the state of a specific telltale
 * 
 * @param telltale The telltale to set
 * @param state The state to set (OFF, ON, FLASH)
 */
 void VX1::SetTelltaleState(Telltale telltale, TelltaleState state)
 {
     // Validate the state (don't allow UNUSED state)
     if (state == TELLTALE_UNUSED)
         state = TELLTALE_OFF;
         
     // Set the appropriate telltale state
     switch (telltale)
     {
         case TELLTALE_WRENCH:
             wrenchState = state;
             break;
         case TELLTALE_BATTERY:
             batteryState = state;
             break;
         case TELLTALE_TEMPERATURE:
             temperatureState = state;
             break;
         case TELLTALE_RESERVED:
             reservedState = state;
             break;
     }
     
     telltaleActive = true;
 }
 
 /**
  * Task to periodically send telltale control messages (call every 10 seconds)
  * 
  * This should be added to the scheduler to maintain telltale states
  * @param canHardware Pointer to the CAN hardware interface
  * @param masterOnly If true, only the master node can send the message (default false)
  */
 void VX1::TelltaleDisplayTask(CanHardware* canHardware, bool masterOnly)
 {
     // Only proceed if telltale control is active and VX1 mode is enabled
     // If masterOnly is true, also check if this is the master node
     if (telltaleActive && IsEnabled() && (!masterOnly || IsMaster()))
     {
         // Send the telltale control message with current states
         SendTelltaleControl(
             wrenchState,
             batteryState,
             temperatureState,
             reservedState,
             canHardware,
             0x4C,  // Default source address for telltale control
             masterOnly
         );
     }
 }

/**
 * Set the clock display message
 * 
 * @param segment1 Rightmost segment (segment 1)
 * @param segment2 Second segment from right
 * @param segment3 Third segment from right
 * @param segment4 Fourth segment from right
 * @param chargerIndicator Charger indicator character (default 0x00 = none)
 */
void VX1::SetClockDisplay(char segment1, char segment2, char segment3, char segment4, char chargerIndicator)
{
    // Set the clock segments
    clockSegments[0] = segment1;
    clockSegments[1] = segment2;
    clockSegments[2] = segment3;
    clockSegments[3] = segment4;
    clockSegments[4] = '\0'; // Ensure null termination
    
    // Set the charger indicator
    clockChargerIndicator = chargerIndicator;
    
    clockActive = true;
}

/**
 * Send a message to the VX1 clock display
 * 
 * @param canHardware Pointer to the CAN hardware interface
 * @param sourceAddress Source address for the J1939 message (default 0xF9 for Diagnostic)
 * @param masterOnly If true, only the master node can send the message (default false)
 * @param override Override control (default true = 0xAA to force display)
 * @return true if message was sent successfully
 */
bool VX1::SendClockMessage(CanHardware* canHardware, uint8_t sourceAddress, bool masterOnly, bool override)
{
    // Check if VX1 mode is enabled, VX1enCanMsg is set to 1, and we have a valid CAN interface
    if (!IsEnabled() || !canHardware || Param::GetInt(Param::VX1enCanMsg) != 1)
        return false;
        
    // If masterOnly is true, check if this is the master node
    if (masterOnly && !IsMaster())
        return false;
    
    // Prepare the J1939 message
    uint32_t data[2] = {0, 0};
    uint8_t* bytes = (uint8_t*)data;
    
    // Format: 4 clock segments, 2 empty segments, charger indicator, override
    bytes[0] = clockSegments[0]; // Segment 1 (rightmost)
    bytes[1] = clockSegments[1]; // Segment 2
    bytes[2] = clockSegments[2]; // Segment 3
    bytes[3] = clockSegments[3]; // Segment 4 (leftmost)
    bytes[4] = ' ';              // Empty segment 5
    bytes[5] = ' ';              // Empty segment 6
    bytes[6] = clockChargerIndicator; // Charger indicator
    bytes[7] = override ? VX1_OVERRIDE_FORCE : VX1_OVERRIDE_NORMAL; // Override control
    
    // Calculate the J1939 29-bit ID
    // Format: Priority (3 bits) | PGN (18 bits) | Source Address (8 bits)
    // Priority 3 (0b011) << 26 | PGN 0x00FEED << 8 | Source Address
    uint32_t j1939Id = (3 << 26) | (VX1_ODOMETER_PGN << 8) | sourceAddress;
    
    // Send the message
    canHardware->Send(j1939Id, data, 8);
    
    return true;
}

/**
 * Task to periodically send the clock message (call every 100ms)
 * 
 * This should be added to the scheduler if continuous display is needed
 * @param canHardware Pointer to the CAN hardware interface
 * @param masterOnly If true, only the master node can send the message (default false)
 */
void VX1::ClockDisplayTask(CanHardware* canHardware, bool masterOnly)
{
    // Only proceed if clock display is active and VX1 mode is enabled
    // If masterOnly is true, also check if this is the master node
    if (clockActive && IsEnabled() && (!masterOnly || IsMaster()))
    {
        SendClockMessage(canHardware, 0xF9, masterOnly, true);
    }
}

/**
 * Boot display task function - handles the boot display sequence
 * This function is called every 100ms by the scheduler
 */
static void BootDisplayTask()
{
    // Only proceed if we're in an active boot display state and VX1enCanMsg is set to 1
    if (bootDisplayState == BOOT_DISPLAY_IDLE || bootDisplayState == BOOT_DISPLAY_DONE || !bootDisplayCanHardware || Param::GetInt(Param::VX1enCanMsg) != 1)
        return;
    
    // Get current time (ms since boot)
    uint32_t currentTime = bootDisplayTimer++;
    
    // State machine for boot display sequence
    switch (bootDisplayState)
    {
        case BOOT_DISPLAY_WAIT:
            // Wait 5 seconds after boot
            if (currentTime >= 50) // 5 seconds (50 * 100ms)
            {
                // Move to next state
                bootDisplayState = BOOT_DISPLAY_TELLTALE;
                bootDisplayTimer = 0;
                
                // Turn on battery telltale
                VX1::SetTelltaleState(VX1::TELLTALE_BATTERY, VX1::TELLTALE_ON);
                VX1::SendTelltaleControl(
                    VX1::TELLTALE_OFF,
                    VX1::TELLTALE_ON,
                    VX1::TELLTALE_OFF,
                    VX1::TELLTALE_OFF,
                    bootDisplayCanHardware,
                    0x4C,
                    false
                );
            }
            break;
            
        case BOOT_DISPLAY_TELLTALE:
            // Show welcome message on odometer LCD
            if (currentTime == 0) // First iteration
            {
                // Set and send the welcome message
                VX1::SetOdometerMessage(" OI BMS");
                VX1::SendOdometerMessage(nullptr, bootDisplayCanHardware, 0xF9, false);
            }
            
            // Send telltale and welcome message periodically
            if (currentTime % 10 == 0) // Every 1 second (10 * 100ms)
            {
                // Refresh telltale
                VX1::SendTelltaleControl(
                    VX1::TELLTALE_OFF,
                    VX1::TELLTALE_ON,
                    VX1::TELLTALE_OFF,
                    VX1::TELLTALE_OFF,
                    bootDisplayCanHardware,
                    0x4C,
                    false
                );
            }
            
            // Send welcome message every 100ms
            VX1::SendOdometerMessage(nullptr, bootDisplayCanHardware, 0xF9, false);
            
            // After 10 seconds, move to next state
            if (currentTime >= 100) // 10 seconds (100 * 100ms)
            {
                bootDisplayState = BOOT_DISPLAY_UDELTA;
                bootDisplayTimer = 0;
            }
            break;
            
        case BOOT_DISPLAY_UDELTA:
            // Show udelta on odometer LCD
            if (currentTime == 0) // First iteration
            {
                // Get udelta value (integer only)
                int udelta = (int)Param::Get(Param::udelta);
                
                // Format the message: "u XXX"
                char message[7];
                sprintf(message, "u %3d  ", udelta);
                
                // Set and send the message
                VX1::SetOdometerMessage(message);
                VX1::SendOdometerMessage(nullptr, bootDisplayCanHardware, 0xF9, false);
            }
            
            // Send telltale and udelta message periodically
            if (currentTime % 10 == 0) // Every 1 second (10 * 100ms)
            {
                // Refresh telltale
                VX1::SendTelltaleControl(
                    VX1::TELLTALE_OFF,
                    VX1::TELLTALE_ON,
                    VX1::TELLTALE_OFF,
                    VX1::TELLTALE_OFF,
                    bootDisplayCanHardware,
                    0x4C,
                    false
                );
            }
            
            // Send udelta message every 100ms
            VX1::SendOdometerMessage(nullptr, bootDisplayCanHardware, 0xF9, false);
            
            // After 5 seconds, move to next state
            if (currentTime >= 50) // 5 seconds (50 * 100ms)
            {
                bootDisplayState = BOOT_DISPLAY_SOC;
                bootDisplayTimer = 0;
            }
            break;
            
        case BOOT_DISPLAY_SOC:
            // Show SOC on odometer LCD
            if (currentTime == 0) // First iteration
            {
                // Get SOC value (integer only)
                int soc = (int)Param::Get(Param::soc);
                
                // Format the message: "s XX"
                char message[7];
                sprintf(message, " s %2d  ", soc);
                
                // Set and send the message
                VX1::SetOdometerMessage(message);
                VX1::SendOdometerMessage(nullptr, bootDisplayCanHardware, 0xF9, false);
            }
            
            // Send telltale and SOC message periodically
            if (currentTime % 10 == 0) // Every 1 second (10 * 100ms)
            {
                // Refresh telltale
                VX1::SendTelltaleControl(
                    VX1::TELLTALE_OFF,
                    VX1::TELLTALE_ON,
                    VX1::TELLTALE_OFF,
                    VX1::TELLTALE_OFF,
                    bootDisplayCanHardware,
                    0x4C,
                    false
                );
            }
            
            // Send SOC message every 100ms
            VX1::SendOdometerMessage(nullptr, bootDisplayCanHardware, 0xF9, false);
            
            // After 5 seconds, move to next state
            if (currentTime >= 50) // 5 seconds (50 * 100ms)
            {
                bootDisplayState = BOOT_DISPLAY_SOH;
                bootDisplayTimer = 0;
            }
            break;
            
        case BOOT_DISPLAY_SOH:
            // Show SOH on odometer LCD
            if (currentTime == 0) // First iteration
            {
                // Get SOH value (integer only)
                int soh = (int)Param::Get(Param::soh);
                
                // Format the message: "h XX"
                char message[7];
                sprintf(message, " h %2d  ", soh);
                
                // Set and send the message
                VX1::SetOdometerMessage(message);
                VX1::SendOdometerMessage(nullptr, bootDisplayCanHardware, 0xF9, false);
            }
            
            // Send telltale and SOH message periodically
            if (currentTime % 10 == 0) // Every 1 second (10 * 100ms)
            {
                // Refresh telltale
                VX1::SendTelltaleControl(
                    VX1::TELLTALE_OFF,
                    VX1::TELLTALE_ON,
                    VX1::TELLTALE_OFF,
                    VX1::TELLTALE_OFF,
                    bootDisplayCanHardware,
                    0x4C,
                    false
                );
            }
            
            // Send SOH message every 100ms
            VX1::SendOdometerMessage(nullptr, bootDisplayCanHardware, 0xF9, false);
            
            // After 5 seconds, finish the sequence
            if (currentTime >= 50) // 5 seconds (50 * 100ms)
            {
                // Turn off telltales
                VX1::SetTelltaleState(VX1::TELLTALE_BATTERY, VX1::TELLTALE_OFF);
                VX1::SendTelltaleControl(
                    VX1::TELLTALE_OFF,
                    VX1::TELLTALE_OFF,
                    VX1::TELLTALE_OFF,
                    VX1::TELLTALE_OFF,
                    bootDisplayCanHardware,
                    0x4C,
                    false
                );
                
                // Clear the display
                VX1::SetOdometerMessage("      ");
                VX1::SendOdometerMessage(nullptr, bootDisplayCanHardware, 0xF9, false);
                
                // Mark as done
                bootDisplayState = BOOT_DISPLAY_DONE;
            }
            break;
            
        default:
            // Should not happen
            bootDisplayState = BOOT_DISPLAY_DONE;
            break;
    }
}

/**
 * Display a welcome screen on boot showing system information
 * 
 * This function should be called after system initialization.
 * It will only execute if VX1BootLCDMsg parameter is set to 1 and only on the master node.
 * 
 * @param canHardware Pointer to the CAN hardware interface
 * @param scheduler Pointer to the scheduler for timed sequences
 */
void VX1::DisplayBootWelcomeScreen(CanHardware* canHardware, Stm32Scheduler* scheduler)
{
    // Only proceed if VX1 mode is enabled, VX1BootLCDMsg is set to 1, VX1enCanMsg is set to 1, and this is the master node
    if (!IsEnabled() || Param::GetInt(Param::VX1BootLCDMsg) != 1 || Param::GetInt(Param::VX1enCanMsg) != 1 || !IsMaster() || !canHardware || !scheduler)
        return;
    
    // Initialize boot display variables
    bootDisplayState = BOOT_DISPLAY_WAIT;
    bootDisplayTimer = 0;
    bootDisplayCanHardware = canHardware;
    bootDisplayStartTime = 0;
    
    // Add the boot display task to the scheduler (100ms interval)
    scheduler->AddTask(BootDisplayTask, 100);
}

/**
 * Send a telltale control message to the VX1 display
 * 
 * @param wrench Wrench icon state
 * @param battery Battery icon state
 * @param temperature Temperature icon state
 * @param reserved Reserved bits state (default OFF)
 * @param canHardware Pointer to the CAN hardware interface
 * @param sourceAddress Source address for the J1939 message (default 0x4C for Charger)
 * @param masterOnly If true, only the master node can send the message (default false)
 * @return true if message was sent successfully
 */
 bool VX1::SendTelltaleControl(
     TelltaleState wrench,
     TelltaleState battery,
     TelltaleState temperature,
     TelltaleState reserved,
     CanHardware* canHardware,
     uint8_t sourceAddress,
     bool masterOnly)
 {
     // Check if VX1 mode is enabled, VX1enCanMsg is set to 1, and we have a valid CAN interface
     if (!IsEnabled() || !canHardware || Param::GetInt(Param::VX1enCanMsg) != 1)
         return false;
         
     // If masterOnly is true, check if this is the master node
     if (masterOnly && !IsMaster())
         return false;
     
     // Prepare the J1939 message
     uint32_t data[2] = {0, 0};
     uint8_t* bytes = (uint8_t*)data;
     
     // Byte 0: Construct the telltale control byte
     // Format: Reserved (bits 7-6) | Temperature (bits 5-4) | Battery (bits 3-2) | Wrench (bits 1-0)
     uint8_t telltaleControl = 0;
     telltaleControl |= (reserved & 0x03) << 6;     // Reserved bits 7-6
     telltaleControl |= (temperature & 0x03) << 4;  // Temperature bits 5-4
     telltaleControl |= (battery & 0x03) << 2;      // Battery bits 3-2
     telltaleControl |= (wrench & 0x03) << 0;       // Wrench bits 1-0
     
     // Set the telltale control byte
     bytes[0] = telltaleControl;
     
     // Other bytes are unused (set to 0)
     bytes[1] = 0;
     bytes[2] = 0;
     bytes[3] = 0;
     bytes[4] = 0;
     bytes[5] = 0;
     bytes[6] = 0;
     bytes[7] = 0;
     
     // Calculate the J1939 29-bit ID
     // Format: Priority (3 bits) | PGN (18 bits) | Source Address (8 bits)
     // Priority 3 (0b011) << 26 | PGN 0x00FECA << 8 | Source Address
     uint32_t j1939Id = (3 << 26) | (VX1_TELLTALE_PGN << 8) | sourceAddress;
     
     // Send the message
     canHardware->Send(j1939Id, data, 8);
     
     return true;
 }