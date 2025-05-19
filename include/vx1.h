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
#include "bmsfsm.h"
#include "stm32scheduler.h"
#include <string.h>

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
     * @brief Check if this node is the master node
     * 
     * @param bmsFsm Pointer to the BmsFsm instance
     * @return true if this is the master node (first node with ID 10 by default)
     */
    static bool IsMaster(BmsFsm* bmsFsm = nullptr);
    
    /**
     * @brief Handle parameter changes related to VX1
     * 
     * @param paramNum The parameter that changed
     */
    static void HandleParamChange(Param::PARAM_NUM paramNum);
    /**
    * @brief Telltale indicators that can be controlled
    */
    enum class TelltaleType {
        WRENCH,
        TEMP,
        BATTERY
    };

    /**
    * @brief Telltale states
    */
    enum class TelltaleState {
        OFF,
        ON,
        BLINKING
    };
    
    /**
     * @brief Structure to hold telltale command information
     */
    struct TelltaleCommand {
        TelltaleType type;
        TelltaleState state;
    };
    /**
    * @brief Set the state of a specific telltale
    * 
    * @param type The telltale type to set
    * @param state The state to set (OFF, ON, BLINKING)
    */
    static void SetTelltaleState(TelltaleType type, TelltaleState state);

    /**
    * @brief Send a telltale control message to the VX1 display
    * 
    * @param canHardware Pointer to the CAN hardware interface
    * @param masterOnly If true, only the master node can send the message (default false)
    * @return true if message was sent successfully
    */
    static bool SendTelltaleControl(
        CanHardware* canHardware,
        bool masterOnly = false
    );

    /**
    * @brief Task to periodically send telltale control messages (call every 10 seconds)
    * 
    * This should be added to the scheduler to maintain telltale states
    * @param canHardware Pointer to the CAN hardware interface
    * @param masterOnly If true, only the master node can send the message (default false)
    */
    static void TelltaleDisplayTask(CanHardware* canHardware, bool masterOnly = false);
    
    /**
     * @brief Set the clock display message
     * 
     * @param segment1 Rightmost segment (segment 1)
     * @param segment2 Second segment from right
     * @param segment3 Third segment from right
     * @param segment4 Fourth segment from right
     * @param chargerIndicator Charger indicator character (default 0x00 = none)
     */
    static void SetClockDisplay(char segment1, char segment2, char segment3, char segment4, char chargerIndicator = 0x00);
    
    /**
     * @brief Send a message to the VX1 clock display
     * 
     * @param canHardware Pointer to the CAN hardware interface
     * @param sourceAddress Source address for the J1939 message (default 0xF9 for Diagnostic)
     * @param masterOnly If true, only the master node can send the message (default false)
     * @param override Override control (default true = 0xAA to force display)
     * @return true if message was sent successfully
     */
    static bool SendClockMessage(CanHardware* canHardware, uint8_t sourceAddress = 0xF9, bool masterOnly = false, bool override = true);
    
    /**
     * @brief Task to periodically send the clock message (call every 100ms)
     * 
     * This should be added to the scheduler if continuous display is needed
     * @param canHardware Pointer to the CAN hardware interface
     * @param masterOnly If true, only the master node can send the message (default false)
     */
    static void ClockDisplayTask(CanHardware* canHardware, bool masterOnly = false);
    
    /**
     * @brief Send a message to the VX1 odometer display
     * 
     * @param message The message to display (max 6 characters)
     * @param canHardware Pointer to the CAN hardware interface
     * @param sourceAddress Source address for the J1939 message (default 0x80)
     * @param masterOnly If true, only the master node can send the message (default false)
     * @return true if message was sent successfully
     */
    static bool SendOdometerMessage(const char* message, CanHardware* canHardware, uint8_t sourceAddress = 0x80, bool masterOnly = false);
    
    /**
     * @brief Task to periodically send the odometer message (call every 100ms)
     * 
     * This should be added to the scheduler if continuous display is needed
     * @param canHardware Pointer to the CAN hardware interface
     * @param masterOnly If true, only the master node can send the message (default false)
     */
    static void OdometerDisplayTask(CanHardware* canHardware, bool masterOnly = false);
    
    /**
     * @brief Set the message to be displayed on the odometer
     * 
     * @param message The message to display (max 6 characters)
     */
    static void SetOdometerMessage(const char* message);
    
    /**
     * @brief Display a welcome screen on boot showing system information
     * 
     * This function should be called after system initialization.
     * It will only execute if VX1BootLCDMsg parameter is set to 1 and only on the master node.
     * 
     * @param canHardware Pointer to the CAN hardware interface
     * @param scheduler Pointer to the scheduler for timed sequences
     * @param bmsFsm Pointer to the BmsFsm instance for master node detection
     */
    static void DisplayBootWelcomeScreen(CanHardware* canHardware, Stm32Scheduler* scheduler, BmsFsm* bmsFsm = nullptr);
    
    /**
     * @brief Check if boot display should be initialized and do so if needed
     *
     * This should be called periodically (e.g., from Ms100Task) to initialize 
     * the boot display sequence when the BMS is properly running.
     * 
     * @param canHardware Pointer to the CAN hardware interface
     * @param scheduler Pointer to the scheduler for timed sequences
     * @param bmsFsm Pointer to the BmsFsm instance for master node detection
     */
    static void CheckAndInitBootDisplay(CanHardware* canHardware, Stm32Scheduler* scheduler, BmsFsm* bmsFsm);

private:
    // Convert ASCII character to 7-segment display code
    static uint8_t CharToSegment(char ch);
    
    static char odometerMessage[7]; // 6 characters + null terminator
    static bool displayActive;
    // Current telltale states
    static TelltaleState wrenchState;
    static TelltaleState tempState;
    static TelltaleState batteryState;
    static bool telltaleActive;
    
    // Clock display data
    static char clockSegments[5]; // 4 segments + null terminator
    static char clockChargerIndicator;
    static bool clockActive;
};

#endif // VX1_H
