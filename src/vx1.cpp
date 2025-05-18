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
#include <libopencm3/stm32/f1/bkp.h>
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
    BOOT_DISPLAY_OI_FLY,
    BOOT_DISPLAY_B_M_S,
    BOOT_DISPLAY_UTOTAL,    // Only if VX1enBootstats = 1
    BOOT_DISPLAY_UDELTA,
    BOOT_DISPLAY_SOC,
    BOOT_DISPLAY_SOH,
    BOOT_DISPLAY_TEMPMIN,   // Only if VX1enBootstats = 1
    BOOT_DISPLAY_TEMPMAX,   // Only if VX1enBootstats = 1
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
    return Param::GetInt(Param::VX1mode) == 1;
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
    if (paramNum == Param::VX1mode)
    {
        // Handle VX1 mode change
        // Note: CAN baud rate changes will only take effect after restart
    }
}

/**
 * Convert an ASCII character to 7-segment display code
 * 
 * @param ch Character to convert
 * @return uint8_t 7-segment code (bits correspond to segments A-G)
 */
uint8_t VX1::CharToSegment(char ch) {
    // 7-segment display mapping
    switch (ch) {
        // Digits
        case '0': return 0x3F; // A B C D E F
        case '1': return 0x06; // B C
        case '2': return 0x5B; // A B G E D
        case '3': return 0x4F; // A B C D G
        case '4': return 0x66; // F G B C
        case '5': return 0x6D; // A F G C D
        case '6': return 0x7D; // A F G E D C
        case '7': return 0x07; // A B C
        case '8': return 0x7F; // A B C D E F G
        case '9': return 0x6F; // A B C D F G

        // Uppercase Letters
        case 'A': return 0x77; // A B C E F G
        case 'B': return 0x7C; // C D E F G
        case 'C': return 0x39; // A D E F
        case 'D': return 0x5E; // B C D E G
        case 'E': return 0x79; // A D E F G
        case 'F': return 0x71; // A E F G
        case 'G': return 0x3D; // A C D E F
        case 'H': return 0x76; // B C E F G
        case 'I': return 0x06; // B C (same as 1)
        case 'J': return 0x1E; // B C D E
        case 'L': return 0x38; // D E F
        case 'M': return 0x37; // A E F C (approximation)
        case 'N': return 0x54; // C E G
        case 'O': return 0x3F; // A B C D E F
        case 'P': return 0x73; // A B E F G
        case 'Q': return 0x67; // A B C F G
        case 'R': return 0x50; // E G
        case 'S': return 0x6D; // A F G C D
        case 'T': return 0x78; // D E F G
        case 'U': return 0x3E; // B C D E F
        case 'V': return 0x3E; // B C D E F (same as U)
        case 'W': return 0x7E; // B C D E F G (approximation)
        case 'X': return 0x76; // B C E F G (same as H)
        case 'Y': return 0x6E; // B C D F G
        case 'Z': return 0x5B; // A B G E D (same as 2)

        // Lowercase Letters (stylized approximations)
        case 'a': return 0x5F; // A B C D E G
        case 'b': return 0x7C; // C D E F G
        case 'c': return 0x58; // D E G
        case 'd': return 0x5E; // B C D E G
        case 'e': return 0x7B; // A B D E F G
        case 'f': return 0x71; // A E F G (same as F)
        case 'g': return 0x6F; // A B C D F G (same as 9)
        case 'h': return 0x74; // C E F G
        case 'i': return 0x04; // C
        case 'j': return 0x0E; // B C D
        case 'l': return 0x30; // E F
        case 'n': return 0x54; // C E G
        case 'o': return 0x5C; // C D E G
        case 'q': return 0x67; // A B C F G (same as Q)
        case 'r': return 0x50; // E G
        case 's': return 0x6D; // A F G C D (same as S)
        case 't': return 0x78; // D E F G
        case 'u': return 0x1C; // C D E
        case 'y': return 0x6E; // B C D F G

        // Symbols
        case '-': return 0x40; // G
        case '_': return 0x08; // D
        case '=': return 0x48; // D G
        case ' ': return 0x00; // Blank
        case '.': return 0x00; // No DP in hardware — OFF

        // Default for unknown characters
        default: return 0x00;  // Blank
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
    uint8_t data[8];
    uint8_t* bytes = data;
    
        // Convert characters to 7-segment display codes and place in correct order
    // According to bootmsg-pgn.md, bytes must be in reverse order: 
    // For "OI FLY": 0x6E 0x38 0x71 0x00 0x06 0x3F 0x00 0xAA
    bytes[0] = CharToSegment(odometerMessage[5]); // Right-most character
    bytes[1] = CharToSegment(odometerMessage[4]);
    bytes[2] = CharToSegment(odometerMessage[3]);
    bytes[3] = CharToSegment(odometerMessage[2]);
    bytes[4] = CharToSegment(odometerMessage[1]);
    bytes[5] = CharToSegment(odometerMessage[0]); // Left-most character
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
 * This function is called with the interval set by VX1msgInterval parameter
 */
static void BootDisplayTask()
{
    // Only proceed if we're in an active boot display state, VX1 mode is enabled, and VX1enCanMsg is set to 1
    if (bootDisplayState == BOOT_DISPLAY_IDLE || bootDisplayState == BOOT_DISPLAY_DONE || 
        !bootDisplayCanHardware || !VX1::IsEnabled() || Param::GetInt(Param::VX1enCanMsg) != 1)
        return;
    
    // Get message interval from parameter
    uint32_t msgInterval = Param::GetInt(Param::VX1msgInterval);
    
    // Get current time 
    uint32_t currentTime = bootDisplayTimer++;
    
    // Calculate iterations needed for timing based on message interval
    uint32_t waitIterations = 10000 / msgInterval; // 10 seconds
    uint32_t shortDisplayIterations = 2000 / msgInterval; // 2 seconds
    uint32_t longDisplayIterations = 5000 / msgInterval; // 5 seconds
    
    // State machine for boot display sequence
    switch (bootDisplayState)
    {
        case BOOT_DISPLAY_WAIT:
            // Wait 10 seconds after boot for scooter electronics to start up
            if (currentTime >= waitIterations) 
            {
                // Move to next state
                bootDisplayState = BOOT_DISPLAY_OI_FLY;
                bootDisplayTimer = 0;
            }
            break;
            
        case BOOT_DISPLAY_OI_FLY:
            // Show "OI FLY" on odometer LCD
            if (currentTime == 0) // First iteration
            {
                // Set and send the message
                VX1::SetOdometerMessage("OI FLY");
                VX1::SendOdometerMessage(nullptr, bootDisplayCanHardware, 0xF9, false);
            }
            
            // Send message every iteration
            VX1::SendOdometerMessage(nullptr, bootDisplayCanHardware, 0xF9, false);
            
            // After 2 seconds, move to next state
            if (currentTime >= shortDisplayIterations)
            {
                bootDisplayState = BOOT_DISPLAY_B_M_S;
                bootDisplayTimer = 0;
            }
            break;
            
        case BOOT_DISPLAY_B_M_S:
            // Show " B M S " on odometer LCD
            if (currentTime == 0) // First iteration
            {
                // Set and send the message
                VX1::SetOdometerMessage(" BMMS ");
                VX1::SendOdometerMessage(nullptr, bootDisplayCanHardware, 0xF9, false);
            }
            
            // Send message every iteration
            VX1::SendOdometerMessage(nullptr, bootDisplayCanHardware, 0xF9, false);
            
            // After 2 seconds, check if we should show stats
            if (currentTime >= shortDisplayIterations)
            {
                // Check if we should show stats
                if (Param::GetInt(Param::VX1enBootstats) == 1)
                {
                    bootDisplayState = BOOT_DISPLAY_UTOTAL;
                }
                else
                {
                    bootDisplayState = BOOT_DISPLAY_DONE;
                }
                bootDisplayTimer = 0;
            }
            break;
            
        case BOOT_DISPLAY_UTOTAL:
            // Show battery pack voltage on odometer LCD
            if (currentTime == 0) // First iteration
            {
                // Get the current battery pack voltage (utotal) - first 5 digits only
                // Example: if utotal value = 138864.7, we need to show "U13886"
                s32fp utotalFixed = Param::Get(Param::utotal);
                float utotalRaw = FP_TOFLOAT(utotalFixed);
                int utotal = (int)utotalRaw;
                
                // Take only first 5 digits for display
                char message[7];
                sprintf(message, "U%d", utotal);
                
                // Set and send the message
                VX1::SetOdometerMessage(message);
                VX1::SendOdometerMessage(nullptr, bootDisplayCanHardware, 0xF9, false);
            }
            
            // Send message every iteration
            VX1::SendOdometerMessage(nullptr, bootDisplayCanHardware, 0xF9, false);
            
            // After 5 seconds, move to next state
            if (currentTime >= longDisplayIterations)
            {
                bootDisplayState = BOOT_DISPLAY_UDELTA;
                bootDisplayTimer = 0;
            }
            break;
            
        case BOOT_DISPLAY_UDELTA:
            // Show udelta on odometer LCD
            if (currentTime == 0) // First iteration
            {
                // Get udelta value (full mV int) - left aligned with 5 more chars after 'd'
                // Example: if udelta value = 21.53125, show "d21   "
                // Example: if udelta value = 128.35, show "d128  "
                s32fp udeltaFixed = Param::Get(Param::udelta);
                float udeltaRaw = FP_TOFLOAT(udeltaFixed);
                int udelta = (int)udeltaRaw;
                
                // Format the message exactly as "dXXX  " (left-aligned) per the specification
                char message[7];
                sprintf(message, "d%-5d", udelta);
                
                // Set and send the message
                VX1::SetOdometerMessage(message);
                VX1::SendOdometerMessage(nullptr, bootDisplayCanHardware, 0xF9, false);
            }
            
            // Send message every iteration
            VX1::SendOdometerMessage(nullptr, bootDisplayCanHardware, 0xF9, false);
            
            // After 5 seconds, move to next state
            if (currentTime >= longDisplayIterations)
            {
                bootDisplayState = BOOT_DISPLAY_SOC;
                bootDisplayTimer = 0;
            }
            break;
            
        case BOOT_DISPLAY_SOC:
            // Show SOC on odometer LCD
            if (currentTime == 0) // First iteration
            {
                // Get SOC value (state of charge) - only first 3 digits before decimal
                // Example: if soc value = 72.15625, show "SOC 72"
                // Example: if soc value = 100.00000, show "SOC100"
                
                // Try a different approach - hardcode the value for now
                // We know from the web interface that the actual value should be 72
                //int soc = 72;
                
                // For debugging - log the various ways we've tried to get the SOC value
                s32fp socFixed = Param::Get(Param::soc);
                //float socFromFixed = FP_TOFLOAT(socFixed);
                //float socFromFloat = Param::GetFloat(Param::soc);
                //float socFromBKP = (float)BKP_DR1 / 100.0f;
                int socmsg = (int)socFixed;
                // Format the message exactly as "SOC XX" or "SOCXXX" based on digits per the specification
                char message[7];
                if (socmsg < 100)
                    sprintf(message, "SOC %2d", socmsg);
                else
                    sprintf(message, "SOC%3d", socmsg);
                
                // Set and send the message
                VX1::SetOdometerMessage(message);
                VX1::SendOdometerMessage(nullptr, bootDisplayCanHardware, 0xF9, false);
            }
            
            // Send message every iteration
            VX1::SendOdometerMessage(nullptr, bootDisplayCanHardware, 0xF9, false);
            
            // After 5 seconds, move to next state
            if (currentTime >= longDisplayIterations)
            {
                bootDisplayState = BOOT_DISPLAY_SOH;
                bootDisplayTimer = 0;
            }
            break;
            
        case BOOT_DISPLAY_SOH:
            // Show SOH on odometer LCD
            if (currentTime == 0) // First iteration
            {
                // Get SOH value (state of health) - only first 3 digits before decimal
                // Example: if soh value = 100.0000, show "SOH100"
                s32fp sohFixed = Param::Get(Param::soh);
                float sohRaw = FP_TOFLOAT(sohFixed);
                int soh = (int)sohRaw;
                
                // Format the message exactly as "SOH XX" or "SOHXXX" based on digits per the specification
                char message[7];
                if (soh < 100)
                    sprintf(message, "SOH %2d", soh);
                else
                    sprintf(message, "SOH%3d", soh);
                
                // Set and send the message
                VX1::SetOdometerMessage(message);
                VX1::SendOdometerMessage(nullptr, bootDisplayCanHardware, 0xF9, false);
            }
            
            // Send message every iteration
            VX1::SendOdometerMessage(nullptr, bootDisplayCanHardware, 0xF9, false);
            
            // After 5 seconds, check if we should show temperature stats
            if (currentTime >= longDisplayIterations)
            {
                bootDisplayState = BOOT_DISPLAY_TEMPMIN;
                bootDisplayTimer = 0;
            }
            break;
            
        case BOOT_DISPLAY_TEMPMIN:
            // Show minimum temperature on odometer LCD
            if (currentTime == 0) // First iteration
            {
                // Get tempmin value (integer only) - temperature in degrees C
                s32fp tempminFixed = Param::Get(Param::tempmin);
                float tempminRaw = FP_TOFLOAT(tempminFixed);
                int tempmin = (int)tempminRaw;
                
                // Format the message as "Lt XX" or "Lt-XX" based on sign per specification
                char message[7];
                if (tempmin >= 0)
                    sprintf(message, "Lt%-4d", tempmin);
                else
                    sprintf(message, "Lt%4d", tempmin);
                
                // Set and send the message
                VX1::SetOdometerMessage(message);
                VX1::SendOdometerMessage(nullptr, bootDisplayCanHardware, 0xF9, false);
            }
            
            // Send message every iteration
            VX1::SendOdometerMessage(nullptr, bootDisplayCanHardware, 0xF9, false);
            
            // After 5 seconds, move to next state
            if (currentTime >= longDisplayIterations)
            {
                bootDisplayState = BOOT_DISPLAY_TEMPMAX;
                bootDisplayTimer = 0;
            }
            break;
            
        case BOOT_DISPLAY_TEMPMAX:
            // Show maximum temperature on odometer LCD
            if (currentTime == 0) // First iteration
            {
                // Get tempmax value (integer only) - temperature in degrees C
                s32fp tempmaxFixed = Param::Get(Param::tempmax);
                float tempmaxRaw = FP_TOFLOAT(tempmaxFixed);
                int tempmax = (int)tempmaxRaw;
                
                // Format the message as "Ht XX" or "Ht-XX" based on sign per specification
                char message[7];
                if (tempmax >= 0)
                    sprintf(message, "Ht%-4d", tempmax);
                else
                    sprintf(message, "Ht%4d", tempmax);
                
                // Set and send the message
                VX1::SetOdometerMessage(message);
                VX1::SendOdometerMessage(nullptr, bootDisplayCanHardware, 0xF9, false);
            }
            
            // Send message every iteration
            VX1::SendOdometerMessage(nullptr, bootDisplayCanHardware, 0xF9, false);
            
            // After 5 seconds, finish the sequence
            if (currentTime >= longDisplayIterations)
            {
                bootDisplayState = BOOT_DISPLAY_DONE;
                bootDisplayTimer = 0;
            }
            break;
            
        case BOOT_DISPLAY_DONE:
            // Send 3 empty messages to properly clear the display
            // Format: 0x18FEEDF9 + 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0xAA
            if (currentTime < 3) // Send 3 empty messages
            {
                // Send empty message with all segment codes as 0x00
                uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, VX1_OVERRIDE_FORCE};
                uint32_t j1939Id = (3 << 26) | (VX1_ODOMETER_PGN << 8) | 0xF9;
                bootDisplayCanHardware->Send(j1939Id, (uint32_t*)data, 8);
            }
            else
            {
                // Mark as done after sending all 3 empty messages
                bootDisplayState = BOOT_DISPLAY_IDLE;
            }
            break;
            
        default:
            // Should not happen
            bootDisplayState = BOOT_DISPLAY_DONE;
            break;
    }
}

/**
 * Check if boot display should be initialized and do so if needed
 *
 * This should be called periodically (e.g., from Ms100Task) to initialize 
 * the boot display sequence when the BMS is properly running.
 * 
 * @param canHardware Pointer to the CAN hardware interface
 * @param scheduler Pointer to the scheduler for timed sequences
 * @param bmsFsm Pointer to the BmsFsm instance for master node detection
 */
void VX1::CheckAndInitBootDisplay(CanHardware* canHardware, Stm32Scheduler* scheduler, BmsFsm* bmsFsm)
{
    static bool bootDisplayInitialized = false;
    
    // Initialize boot display screen once, when BMS is running
    if (!bootDisplayInitialized && bmsFsm != nullptr)
    {
        bootDisplayInitialized = true;
        DisplayBootWelcomeScreen(canHardware, scheduler, bmsFsm);
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
 * @param bmsFsm Pointer to the BmsFsm instance for master node detection
 */
void VX1::DisplayBootWelcomeScreen(CanHardware* canHardware, Stm32Scheduler* scheduler, BmsFsm* bmsFsm)
{
    // Only proceed if VX1 mode is enabled, VX1BootLCDMsg is set to 1, VX1enCanMsg is set to 1, and this is the master node
    if (!IsEnabled() || Param::GetInt(Param::VX1BootLCDMsg) != 1 || Param::GetInt(Param::VX1enCanMsg) != 1 || !IsMaster(bmsFsm) || !canHardware || !scheduler)
        return;
    
    // Initialize boot display variables
    bootDisplayState = BOOT_DISPLAY_WAIT;
    bootDisplayTimer = 0;
    bootDisplayCanHardware = canHardware;
    bootDisplayStartTime = 0;
    
    // Get message interval from parameter
    uint32_t msgInterval = Param::GetInt(Param::VX1msgInterval);
    
    // Add the boot display task to the scheduler with the configured interval
    scheduler->AddTask(BootDisplayTask, msgInterval);
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