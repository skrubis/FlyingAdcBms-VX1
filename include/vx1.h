/*
 * This file is part of the FlyingAdcBms-VX1 project.
 *
 * Copyright (C) 2025 SKRUBIS (https://github.com/skrubis)
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
//  This project is provided as-is, without any express or implied warranties, including but not limited to warranties of merchantability, fitness for a particular purpose, or non-infringement.
//  All CAN bus messages and PGNs referenced or used in this project are either:
//  Defined in the public SAE J1939 standard,
//  Commonly available through public forums, technical documentation, and community research, or
//  Independently observed through legal means without bypassing any security mechanisms or protections.
//  No proprietary firmware, OEM software, or copyrighted code has been copied, reverse-engineered, or distributed.
//  This project is intended solely for educational, research, and interoperability purposes. It is not certified for use in production vehicles or safety-critical systems.
//  Use of this project is entirely at your own risk. Interfacing with vehicle systems—especially those involving power electronics or battery management—can result in:
//  Vehicle malfunction or damage
//  Battery failure or fire
//  Injury or death
//  Voiding of warranties or violation of local laws
//  By using or modifying this project, you accept full responsibility for any consequences that arise. The authors shall not be held liable for any direct, indirect, incidental, special, or consequential damages resulting from the use, misuse, or inability to use this project or any derivative works.
//  If you believe that any content herein infringes upon your legal rights or intellectual property, please contact the authors with specific details. We are committed to addressing any valid concerns in a respectful and timely manner.


#ifndef VX1_H
#define VX1_H

#include "params.h"
#include "canhardware.h"
#include "bmsfsm.h"
#include "stm32scheduler.h"
#include "errormessage.h"
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
     * @brief Error message short codes for display on odometer
     */
    struct ErrorShortCode {
        ERROR_MESSAGE_NUM errorCode;
        const char* shortCode;
    };
    
    // Error short code lookup table for the LCD display
    static const ErrorShortCode ERROR_SHORT_CODES[];
    
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
     * Task to periodically send the clock message (call every 100ms)
     * 
     * This should be added to the scheduler if continuous display is needed
     * @param canHardware Pointer to the CAN hardware interface
     * @param masterOnly If true, only the master node can send the message (default false)
     */
    static void ClockDisplayTask(CanHardware* canHardware, bool masterOnly = false);
    
    /**
     * Display user configurable stats on the clock display
     * 
     * This function checks if conditions are met to display stats on the clock display:
     * - VX1mode must be 1 (enabled)
     * - VX1LCDClockStats must be 1 (always) or 2 (only when idle)
     * - If VX1LCDClockStats is 2, idlecurrent > idcavg must be true
     * 
     * @param canHardware Pointer to the CAN hardware interface
     * @param bmsFsm Pointer to the BmsFsm instance for master node detection
     */
    static void ClockStatsDisplayTask(CanHardware* canHardware, BmsFsm* bmsFsm = nullptr);
    
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
     * @brief Report an error by showing blinking battery and wrench telltales and error message on odometer
     * 
     * @param errorCode The error code to report
     * @param nodeId The node ID of the BMS reporting the error
     * @param canHardware Pointer to the CAN hardware interface
     * @return true if the error message was sent successfully
     */
    static bool ReportError(ERROR_MESSAGE_NUM errorCode, uint8_t nodeId, CanHardware* canHardware);
    
    /**
     * @brief Error reporting task - should be called periodically
     * 
     * @param canHardware Pointer to the CAN hardware interface
     * @param bmsFsm Pointer to the BmsFsm instance
     */
    static void ErrorReportingTask(CanHardware* canHardware, BmsFsm* bmsFsm);
    
    /**
     * @brief Report temperature warning by blinking battery telltale and showing tempmax on odometer
     * 
     * @param temperature The temperature value to report
     * @param canHardware Pointer to the CAN hardware interface
     * @return true if the warning message was sent successfully
     */
    static bool ReportTemperatureWarning(float temperature, CanHardware* canHardware);
    
    /**
     * @brief Temperature warning task - should be called periodically
     * 
     * @param canHardware Pointer to the CAN hardware interface
     * @param bmsFsm Pointer to the BmsFsm instance
     */
    static void TemperatureWarningTask(CanHardware* canHardware, BmsFsm* bmsFsm);
    
    /**
     * @brief Report voltage delta warning by turning on solid wrench telltale and showing udelta on odometer
     * 
     * @param uDelta The voltage delta value to report
     * @param canHardware Pointer to the CAN hardware interface
     * @return true if the warning message was sent successfully
     */
    static bool ReportUDeltaWarning(float uDelta, CanHardware* canHardware);
    
    /**
     * @brief Voltage delta warning task - should be called periodically
     * 
     * @param canHardware Pointer to the CAN hardware interface
     * @param bmsFsm Pointer to the BmsFsm instance
     */
    static void UDeltaWarningTask(CanHardware* canHardware, BmsFsm* bmsFsm);
    
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

    /**
     * @brief Process CAN messages with PGN FEF1h from SA 0x05 (vehicle data)
     * 
     * This function extracts speed, voltage, and current data from the message
     * and updates the corresponding parameters.
     * 
     * @param canId CAN ID of the received message
     * @param data Array containing the message data
     */
    static void ProcessVehicleDataMessage(uint32_t canId, uint32_t data[2]);
    
    /**
     * @brief Calculate kWh per 100km based on accumulated energy and distance
     * 
     * This function should be called periodically to update the consumption calculation
     */
    static void UpdateEnergyConsumption();
    
    /**
     * @brief Register for receiving vehicle data messages
     * 
     * @param canHardware Pointer to the CAN hardware interface
     */
    static void RegisterVehicleDataMessages(CanHardware* canHardware);
    
    /**
     * @brief Send BMS PGN messages if conditions are met
     * 
     * Sends BMS PGN messages according to bms-comms.md specification
     * Only sends if VX1mode=1, VX1enCanMsg=1, VX1EmulateBMSmsg=1, and node is master
     * 
     * @param canHardware Pointer to the CAN hardware interface
     * @param bmsFsm Pointer to the BmsFsm instance
     */
    static void BmsPgnEmulationTask(CanHardware* canHardware, BmsFsm* bmsFsm);
    
    /**
     * @brief Send BMS Status & Control PGN (0xFEF2)
     * 
     * Maps parameters and sends BMS status information including SoC, temperatures,
     * voltage, operational states, and various requests.
     * 
     * @param canHardware Pointer to the CAN hardware interface
     * @return true if message was sent successfully
     */
    static bool SendBmsPgn0xFEF2(CanHardware* canHardware);
    
    /**
     * @brief Send Cell Voltage and Temperature Extremes PGN (0xFEF3)
     * 
     * Maps cell voltage parameters to CAN message fields according to specification.
     * 
     * @param canHardware Pointer to the CAN hardware interface
     * @return true if message was sent successfully
     */
    static bool SendBmsPgn0xFEF3(CanHardware* canHardware);
    
    /**
     * @brief Send Faults, Status Flags, and Maintenance Codes PGN (0xFEF4)
     * 
     * Maps warning and error states to CAN message fields according to specification.
     * 
     * @param canHardware Pointer to the CAN hardware interface
     * @return true if message was sent successfully
     */
    static bool SendBmsPgn0xFEF4(CanHardware* canHardware);
    
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
    static uint32_t lastTelltaleUpdateTime; // Store last time telltales were updated
    
    // Clock display data
    static char clockSegments[5]; // 4 segments + null terminator
    static char clockChargerIndicator;
    static bool clockActive;
    
    // Error reporting states
    static bool errorActive;
    static ERROR_MESSAGE_NUM currentError;
    static uint8_t errorNodeId;
    
    // Temperature warning states
    static bool tempWarningActive;
    static float currentTempWarning;
    
    // Voltage delta warning states
    static bool uDeltaWarningActive;
    static float currentUDeltaWarning;
    
    // Vehicle data from PGN FEF1h
    static float vehicleSpeed;     // Current speed in km/h
    static float busVoltage;       // Bus voltage in V
    static float busCurrent;       // Bus current in A
    static uint32_t lastVehicleDataTime; // Last time vehicle data was received
    
    // Energy consumption calculation variables
    static float totalEnergyWh;    // Total energy consumed in Wh
    static float totalDistanceKm;  // Total distance traveled in km
    static float kWhPer100km;      // Calculated consumption in kWh/100km
    static uint32_t lastCalculationTime; // Last time consumption was calculated
};

#endif // VX1_H
