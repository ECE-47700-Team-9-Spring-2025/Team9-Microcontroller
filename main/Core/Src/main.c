/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "m8q.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

static uint8_t rx_buf[20];
static char* tx_1 = "AT";
// Define setup commands for HM-10 Bluetooth module
static char* setup_cmds[] = {
    "AT+ROLE0",      // Set as peripheral
    "AT+ADVI3",      // Set advertising interval to 318.75ms (better compatibility)
    "AT+ADTY0",      // Allow advertising and connections
    "AT+FLAG1",      // Enable advertising flag (critical for iOS visibility)
    "AT+NAMEFairway",
    "AT+SHOW1",
    "AT+IBEA0",
    "AT+UUID0xFFE0", // Set standard service UUID
    "AT+CHAR0xFFE1", // Set standard characteristic UUID
    "AT+POWE3",      // Max transmit power (6dbm)
    "AT+RESET",      // Reboot module
    "AT+NAME?"       // Verify name
};

// ICM-20948 Register addresses - Updated for correct bank 0 addresses
#define WHO_AM_I_REG     0x00    
#define WHO_AM_I_VAL     0xEA   

#define PWR_MGMT_1       0x06
#define ACCEL_XOUT_H     0x2D
#define ACCEL_XOUT_L     0x2E
#define ACCEL_YOUT_H     0x2F
#define ACCEL_YOUT_L     0x30
#define ACCEL_ZOUT_H     0x31
#define ACCEL_ZOUT_L     0x32
#define GYRO_XOUT_H      0x33
#define GYRO_XOUT_L      0x34
#define GYRO_YOUT_H      0x35
#define GYRO_YOUT_L      0x36
#define GYRO_ZOUT_H      0x37
#define GYRO_ZOUT_L      0x38
#define PWR_MGMT_2       0x07
#define GYRO_CONFIG_1    0x01
#define ACCEL_CONFIG     0x14

// ICM-20948 specific defines
#define ICM_CS_PIN       GPIO_PIN_1
#define ICM_CS_PORT      GPIOB

// Add these defines for magnetometer registers (AK09916)
#define MAG_WHO_AM_I        0x01  // Should return 0x09
#define MAG_ST1             0x10  // Status 1
#define MAG_HXL             0x11  // X-axis LSB
#define MAG_HXH             0x12  // X-axis MSB
#define MAG_HYL             0x13  // Y-axis LSB
#define MAG_HYH             0x14  // Y-axis MSB
#define MAG_HZL             0x15  // Z-axis LSB
#define MAG_HZH             0x16  // Z-axis MSB
#define MAG_ST2             0x18  // Status 2
#define MAG_CNTL2           0x31  // Control 2
#define MAG_CNTL3           0x32  // Control 3
#define USER_BANK_SEL	(0x7F)
#define USER_BANK_0		(0x00)
#define USER_BANK_1		(0x10)
#define USER_BANK_2		(0x20)
#define USER_BANK_3		(0x30)
#define CLK_BEST_AVAIL	(0x01)
#define GYRO_RATE_250	(0x00)
#define GYRO_LPF_17HZ 	(0x29)

// Magnetometer data storage
int16_t mag_data[3];

// Define magnetic declination for your location
// This is the angle between magnetic north and true north
// Look up the value for your area: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
#define MAGNETIC_DECLINATION_DEG -4.48f  // Purdue University's Magnetic Declination
#define DEBUG_GPS_DATA 1
#define DEBUG_IMU_DATA 0

// Motor pins (update these based on your hardware connections)
#define MOTOR_LEFT_FWD_TIM       htim2
#define MOTOR_LEFT_FWD_CHANNEL   TIM_CHANNEL_1
#define MOTOR_LEFT_REV_TIM       htim2
#define MOTOR_LEFT_REV_CHANNEL   TIM_CHANNEL_2

#define MOTOR_RIGHT_FWD_TIM      htim3
#define MOTOR_RIGHT_FWD_CHANNEL  TIM_CHANNEL_1
#define MOTOR_RIGHT_REV_TIM      htim3
#define MOTOR_RIGHT_REV_CHANNEL  TIM_CHANNEL_2

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    GPS_STATUS_INIT,
    GPS_STATUS_OK,
    GPS_STATUS_ERROR,
    GPS_STATUS_NO_FIX
} GPS_Status;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */
GPS_Data gps_data;
GPS_Data phone_gps_data;
GPS_Data previous_gps_data;       // Add this to store previous GPS data
GPS_Data previous_phone_gps_data; // Add this to store previous phone GPS data
GNSSVector gnss_vector;           // Store the calculated vector

// DMA buffer for UART reception
#define UART_RX_BUFFER_SIZE 512
uint8_t uartRxBuffer[UART_RX_BUFFER_SIZE];
volatile uint16_t rxHead = 0;
volatile uint16_t searchPos = 0;

// Add this for Bluetooth GPS reception - 26 bytes as per the app's specification
#define BT_GPS_DATA_SIZE 26
uint8_t bt_gps_buffer[BT_GPS_DATA_SIZE];
volatile bool bt_gps_data_ready = false;

// Add these global variables for rolling average
#define ROLLING_AVG_SAMPLES 3
static int16_t accel_history[ROLLING_AVG_SAMPLES][3] = {0};
static int16_t mag_history[ROLLING_AVG_SAMPLES][3] = {0};
static uint8_t history_index = 0;

// Add this global variable to track when to process GPS data
static uint32_t lastGpsProcessTime = 0;
#define GPS_PROCESS_INTERVAL 1000 // Process GPS data every 1 second
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI5_Init(void);
/* USER CODE BEGIN PFP */
float bytesToFloat(uint8_t* bytes);
void parsePhoneGPSData(uint8_t* buffer, GPS_Data* gps);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void printToConsole(const char *format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void decodeNMEASentence(const char *sentence) {
    char buffer[128];
    strncpy(buffer, sentence, sizeof(buffer));
    buffer[sizeof(buffer)-1] = '\0';
    
    char *token = strtok(buffer, ",");
    int fieldIndex = 0;
    
    while (token != NULL) {
        printToConsole("Field %d: %s\r\n", fieldIndex, token);
        token = strtok(NULL, ",");
        fieldIndex++;
    }
}

bool readUntilNewline(char *buffer, size_t maxSize) {
    uint32_t startTick = HAL_GetTick();
    uint32_t idx = 0;
    char ch;
    bool startFound = false;
    
    // Clear buffer first
    memset(buffer, 0, maxSize);
    
    while (idx < (maxSize - 1)) {
        // Check for timeout - 2 seconds should be enough for one complete sentence
        if ((HAL_GetTick() - startTick) > 2000) {
            if (idx > 0) {
                printToConsole("Timeout waiting for complete sentence after %d chars\r\n", idx);
            }
            break;
        }
        
        HAL_StatusTypeDef status = HAL_UART_Receive(&huart6, (uint8_t*)&ch, 1, 100);
        
        if (status == HAL_OK) {
            // Debug - uncomment if needed
            // printToConsole("%c", ch);
            
            // If we find the start character ($), reset buffer and start collecting
            if (ch == '$') {
                startFound = true;
                idx = 0;
                buffer[idx++] = ch;
                continue;
            }
            
            // Only add characters if we've found the start
            if (startFound) {
                buffer[idx++] = ch;
                
                // Check for complete NMEA sentence: CR+LF ending
                if (idx >= 2 && ch == '\n' && buffer[idx-2] == '\r') {
                    buffer[idx] = '\0';
                    return true;
                }
                
                // Alternative check: we found a complete sentence with checksum
                // Some devices might not properly terminate with CR+LF
                if (idx >= 5 && buffer[idx-3] == '*') {  // Found checksum marker
                    // Allow 2 more chars for the checksum itself
                    if (idx >= (uint32_t)(buffer[idx-3] + 3)) {
                        // Check if next char is CR or LF - if so, we're done
                        status = HAL_UART_Receive(&huart6, (uint8_t*)&ch, 1, 10);
                        if (status == HAL_OK) {
                            if (ch == '\r' || ch == '\n') {
                                buffer[idx++] = ch;
                                // Get the matching LF if we found CR
                                if (ch == '\r') {
                                    status = HAL_UART_Receive(&huart6, (uint8_t*)&ch, 1, 10);
                                    if (status == HAL_OK && ch == '\n') {
                                        buffer[idx++] = ch;
                                    }
                                }
                                buffer[idx] = '\0';
                                return true;
                            } else {
                                // Not CR/LF, add to buffer and continue
                                buffer[idx++] = ch;
                            }
                        } else {
                            // No more data but we have a complete sentence with checksum
                            buffer[idx] = '\0';
                            return true;
                        }
                    }
                }
            }
        } else if (status == HAL_TIMEOUT && startFound && idx > 5) {
            // We've started a sentence and got something meaningful
            buffer[idx] = '\0';
            return true;
        }
    }
    
    // If we've collected a valid-looking sentence but hit the buffer limit
    if (startFound && idx > 0) {
        buffer[idx] = '\0';
        return true;
    }
    
    buffer[0] = '\0';
    return false;
}

GPS_Status parseGPSTXT(const char* message) {
    if (strstr(message, "ANTSTATUS=INIT")) {
        return GPS_STATUS_INIT;
    } else if (strstr(message, "ANTSTATUS=OK")) {
        return GPS_STATUS_OK;
    } else if (strstr(message, "ANTSTATUS=SHORT") || 
               strstr(message, "ANTSTATUS=OPEN")) {
        return GPS_STATUS_ERROR;
    }
    return GPS_STATUS_NO_FIX;
}

bool printCurrentGpsOutput(void) {
    char buffer[256];
    static GPS_Status lastStatus = GPS_STATUS_INIT;
    static uint32_t noFixCount = 0;
    static uint32_t lastDebugPrint = 0;
    const uint32_t DEBUG_PRINT_INTERVAL = 1000; // Print debug every 1 second
    
    memset(buffer, 0, sizeof(buffer));
    memset(&gps_data, 0, sizeof(GPS_Data));

    if (readUntilNewline(buffer, sizeof(buffer))) {
        // Debug message with timestamp
        uint32_t currentTick = HAL_GetTick();
        if (currentTick - lastDebugPrint >= DEBUG_PRINT_INTERVAL) {
            printToConsole("\r\n=== GPS Debug [%lu ms] ===\r\n", currentTick);
            lastDebugPrint = currentTick;
        }

        // Validate NMEA message format
        if (buffer[0] != '$') {
            printToConsole("ERROR: Invalid NMEA format\r\n");
            return false;
        }
        
        // Test buffer
        // const char* buffer = "$GNRMC,201850.00,A,4025.69979,N,08654.69118,W,0.256,,240225,08654.69118,W,0.256,,240225,,,A*73";
        
        // Parse message type
        if (strstr(buffer, "$GNRMC")) {
            printToConsole("Message Type: RMC (Position/Speed/Time)\r\n");
            
            if (M8Q_ParseGNRMC(buffer, &gps_data)) {
                if (!gps_data.fix_valid) {
                    noFixCount++;
                    printToConsole("Status: NO FIX (Waiting: %lu sec)\r\n", noFixCount);
                    printToConsole("Time: %02d:%02d:%02d UTC\r\n", 
                        gps_data.hours, 
                        gps_data.minutes, 
                        gps_data.seconds);
                    printToConsole("Troubleshooting:\r\n");
                    printToConsole("- Ensure clear view of sky\r\n");
                    printToConsole("- Wait for satellite acquisition (can take 1-5 min)\r\n");
                    printToConsole("- Check antenna connection\r\n");
                } else {
                    noFixCount = 0;
                    printToConsole("\r\n=== GPS Location Update ===\r\n");
                    printToConsole("Time: %02d:%02d:%02d UTC\r\n", 
                        gps_data.hours, 
                        gps_data.minutes, 
                        gps_data.seconds);
                    
                    printToConsole("Date: %02d/%02d/%04d\r\n", 
                        gps_data.day, 
                        gps_data.month, 
                        gps_data.year);
                    
                    // Convert coordinates to degrees and decimal minutes format
                    int lat_deg = (int)gps_data.latitude;
                    double lat_min = (gps_data.latitude - lat_deg) * 60;
                    int lon_deg = (int)gps_data.longitude;
                    double lon_min = (gps_data.longitude - lon_deg) * 60;
                    
                    printToConsole("Position:\r\n");
                    printToConsole("  %d°%.4f' %c\r\n", 
                        abs(lat_deg), fabs(lat_min), gps_data.lat_direction);
                    printToConsole("  %d°%.4f' %c\r\n", 
                        abs(lon_deg), fabs(lon_min), gps_data.lon_direction);
                    
                    if (gps_data.speed_knots > 0.5) { // Only show speed if moving
                        printToConsole("Speed: %.1f km/h\r\n", 
                            gps_data.speed_knots * 1.852); // Convert knots to km/h
                        printToConsole("Heading: %.1f°\r\n", 
                            gps_data.course);
                    }
                    
                    printToConsole("=========================\r\n");
                }
            } else {
                printToConsole("ERROR: Failed to parse RMC message\r\n");
                printToConsole("Raw: %s\r\n", buffer);
            }
            return true;
        } 
        else if (strstr(buffer, "$GNGGA")) {
            printToConsole("Message Type: GGA (GPS Fix Data)\r\n");
            
            // Parse GGA message fields
            char *saveptr;
            char *token = strtok_r(buffer, ",", &saveptr);
            int field = 0;
            
            while (token != NULL) {
                switch(field) {
                    case 6: // Fix quality
                        printToConsole("Fix Quality: ");
                        switch(atoi(token)) {
                            case 0: printToConsole("Invalid\r\n"); break;
                            case 1: printToConsole("GPS Fix\r\n"); break;
                            case 2: printToConsole("DGPS Fix\r\n"); break;
                            default: printToConsole("Unknown (%s)\r\n", token); break;
                        }
                        break;
                    case 7: // Satellites in use
                        printToConsole("Satellites: %s in use\r\n", token);
                        break;
                    case 8: // HDOP
                        {
                            float hdop = atof(token);
                            printToConsole("HDOP: %.1f ", hdop);
                            if (hdop < 1.0) printToConsole("(Excellent)\r\n");
                            else if (hdop < 2.0) printToConsole("(Good)\r\n");
                            else if (hdop < 5.0) printToConsole("(Moderate)\r\n");
                            else printToConsole("(Poor)\r\n");
                        }
                        break;
                }
                field++;
                token = strtok_r(NULL, ",", &saveptr);
            }
            return true;
        }
        else if (strstr(buffer, "$GNTXT")) {
            GPS_Status status = parseGPSTXT(buffer);
            if (status != lastStatus) {
                switch(status) {
                    case GPS_STATUS_INIT:
                        printToConsole("GPS Status: Initializing antenna\r\n");
                        break;
                    case GPS_STATUS_OK:
                        printToConsole("GPS Status: Antenna OK, waiting for fix\r\n");
                        break;
                    case GPS_STATUS_ERROR:
                        printToConsole("GPS Status: Antenna error detected!\r\n");
                        break;
                    default:
                        break;
                }
                lastStatus = status;
            }
            printToConsole("GPS Info Message: %s", buffer);
            return true;
        }
        else {
            printToConsole("Message Type: Other (%.*s)\r\n", 5, buffer);
            return true;
        }
    } else {
        printToConsole("ERROR: Failed to read NMEA sentence\r\n");
        return false;
    }
}


void resetBluetoothModule(void) {
    // 1. Send software reset command
    char reset_cmd[] = "AT+RESET\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)reset_cmd, strlen(reset_cmd), HAL_MAX_DELAY);
    
    // 2. Wait for full reboot (critical!)
    HAL_Delay(2000);  // HM-10 needs at least 1.5 seconds to reboot
    
    // 3. Clear all existing configurations
    char clear_cmd[] = "AT+RENEW\r\n";  // Restore factory defaults
    HAL_UART_Transmit(&huart1, (uint8_t*)clear_cmd, strlen(clear_cmd), HAL_MAX_DELAY);
    HAL_Delay(1000);
    
    // 4. Clear receive buffer
    memset(rx_buf, 0, sizeof(rx_buf));
}


// Function to initialize HM-10 BLE module
void initBluetooth(void) {
    printToConsole("\r\n== Starting HM-10 Bluetooth Initialization ==\r\n");
    uint8_t response_received = 0;
    
    // Initial check - send AT to see if module responds
    printToConsole("Sending test command: AT\r\n");
    int size = strlen(tx_1);
    HAL_UART_Receive_DMA(&huart1, rx_buf, size);
    HAL_UART_Transmit(&huart1, (uint8_t*)tx_1, size, HAL_MAX_DELAY);
    
    // Wait for response with timeout
    uint32_t startTime = HAL_GetTick();
    while (!response_received && (HAL_GetTick() - startTime < 5000)) {
        if (strstr((char*)rx_buf, "OK")) {
            response_received = 1;
            printToConsole("\r\nBLE module responded OK to test command\r\n");
        }
        HAL_Delay(50);
    }
    
    if (!response_received) {
        printToConsole("ERROR: No response from BLE module! Check connections\r\n");
        return;
    }
    
    // Process each setup command
    for (int i = 0; i < sizeof(setup_cmds)/sizeof(setup_cmds[0]); i++) {
        // Cancel any ongoing reception
        HAL_UART_AbortReceive(&huart1);

        // Clear response buffer
        memset(rx_buf, 0, sizeof(rx_buf));
        response_received = 0;
        
        // Get command length
        size = strlen(setup_cmds[i]);
        
        printToConsole("Sending: %s\r\n", setup_cmds[i]);
        
        // Start reception before sending command
        HAL_UART_Receive_DMA(&huart1, rx_buf, sizeof(rx_buf));
        HAL_UART_Transmit(&huart1, (uint8_t*)setup_cmds[i], size, HAL_MAX_DELAY);
        
        // Wait for response with timeout (slightly longer for RESET command)
        int timeout = (strstr(setup_cmds[i], "RESET") != NULL) ? 10000 : 10000;
        startTime = HAL_GetTick();
        
        while (!response_received && (HAL_GetTick() - startTime < timeout)) {
            if (strstr((char*)rx_buf, "OK")) {
                response_received = 1;
                printToConsole("Response: %s\r\n", rx_buf);
            }
            HAL_Delay(50);
        }
        
        if (!response_received) {
            printToConsole("WARNING: No response to command: %s\r\n", setup_cmds[i]);
        }
        
        // Add delay between commands
        HAL_Delay(1000);
    }
    
    printToConsole("== HM-10 Bluetooth Initialization Complete ==\r\n");
}

bool getNMEASentence(char *buffer, size_t maxSize) {
    uint16_t startPos = UINT16_MAX;
    uint16_t endPos = UINT16_MAX;
    uint16_t currentHead = rxHead; // Capture current position
    uint16_t pos = searchPos;
    uint16_t searchEndPos = (currentHead >= searchPos) ? currentHead : (currentHead + UART_RX_BUFFER_SIZE);
    
    // Search for complete NMEA sentence
    while (pos < searchEndPos) {
        uint16_t bufferPos = pos % UART_RX_BUFFER_SIZE;
        
        // Look for sentence start
        if (uartRxBuffer[bufferPos] == '$' && startPos == UINT16_MAX) {
            startPos = bufferPos;
        }
        // Look for sentence end (CR+LF or just LF)
        else if (startPos != UINT16_MAX && uartRxBuffer[bufferPos] == '\n') {
            endPos = bufferPos;
            break;
        }
        pos++;
    }
    
    // If complete sentence found
    if (startPos != UINT16_MAX && endPos != UINT16_MAX) {
        uint16_t length = 0;
        pos = startPos;
        
        // Calculate sentence length accounting for buffer wrap
        uint16_t sentenceLength = (endPos >= startPos) ? 
            (endPos - startPos + 1) : 
            (UART_RX_BUFFER_SIZE - startPos + endPos + 1);
            
        // Check if sentence fits in output buffer
        if (sentenceLength >= maxSize) {
            searchPos = (endPos + 1) % UART_RX_BUFFER_SIZE;
            return false;
        }
        
        // Copy sentence to output buffer
        while (length < sentenceLength && length < maxSize - 1) {
            buffer[length++] = uartRxBuffer[pos];
            pos = (pos + 1) % UART_RX_BUFFER_SIZE;
        }
        
        buffer[length] = '\0';
        searchPos = (endPos + 1) % UART_RX_BUFFER_SIZE;
        
        // Validate basic NMEA format
        if (length > 6 && buffer[0] == '$' && 
            (buffer[length-2] == '\r' || buffer[length-1] == '\n')) {
            return true;
        }
    }
    
    // If we've searched the entire new data without finding a sentence,
    // move search position to avoid re-searching
    if (pos >= searchEndPos) {
        searchPos = currentHead;
    }
    
    return false;
}

// New function to process GPS data
void ProcessGpsData() {
    char nmeaBuffer[256];
    
    if (getNMEASentence(nmeaBuffer, sizeof(nmeaBuffer))) {
        // Debug raw NMEA sentence
        if (DEBUG_GPS_DATA) {
            printToConsole("\r\n--- Raw NMEA Sentence ---\r\n");
            printToConsole("Length: %d bytes\r\n", strlen(nmeaBuffer));
            printToConsole("Content: %s", nmeaBuffer);
        }
        
        if (strstr(nmeaBuffer, "$GNRMC")) {
            printToConsole("=== GNRMC Message Detected ===\r\n");  
            
            // Store previous GPS data before updating
            memcpy(&previous_gps_data, &gps_data, sizeof(GPS_Data));
            
            bool success = M8Q_ParseGNRMC(nmeaBuffer, &gps_data);
            if (success) {
                printToConsole("\r\nParsing Successful!\r\n");
                printToConsole("Time: %02d:%02d:%02d UTC\r\n", 
                    gps_data.hours, gps_data.minutes, gps_data.seconds);
                printToConsole("Fix Valid: %s\r\n", 
                    gps_data.fix_valid ? "Yes" : "No");
                printToConsole("Position: %.6f%c, %.6f%c\r\n",
                    gps_data.latitude, gps_data.lat_direction,
                    gps_data.longitude, gps_data.lon_direction);
                if (gps_data.speed_knots > 0) {
                    printToConsole("Speed: %.2f knots\r\n", gps_data.speed_knots);
                    printToConsole("Course: %.2f degrees\r\n", gps_data.course);
                }
            }
        } 

        if (strstr(nmeaBuffer, "$GNGLL")) {
            printToConsole("=== GNGLL Message Detected ===\r\n");

            // Store previous GPS data before updating
            memcpy(&previous_gps_data, &gps_data, sizeof(GPS_Data));
            
            bool success = M8Q_ParseGNGLL(nmeaBuffer, &gps_data);
            if (success) {
                printToConsole("\r\nParsing Successful!\r\n");
                printToConsole("Time: %02d:%02d:%02d UTC\r\n", 
                    gps_data.hours, gps_data.minutes, gps_data.seconds);
                printToConsole("Fix Valid: %s\r\n", 
                    gps_data.fix_valid ? "Yes" : "No");
                printToConsole("Position: %.6f%c, %.6f%c\r\n",
                    gps_data.latitude, gps_data.lat_direction,
                    gps_data.longitude, gps_data.lon_direction);
                if (gps_data.speed_knots > 0) {
                    printToConsole("Speed: %.2f knots\r\n", gps_data.speed_knots);
                    printToConsole("Course: %.2f degrees\r\n", gps_data.course);
                }
            }
        }
    }
    //  else {
    //     printToConsole("No Microcontroller GPS data present! Please check the connection.\r\n");
    // }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART6) {
        // Calculate the new head position
        uint16_t newHead = (rxHead + Size) % UART_RX_BUFFER_SIZE;
        rxHead = newHead;
        
        // If searchPos is far behind, move it forward to avoid searching old data
        if ((rxHead > searchPos && (rxHead - searchPos) > UART_RX_BUFFER_SIZE/2) ||
            (rxHead < searchPos && (UART_RX_BUFFER_SIZE - searchPos + rxHead) > UART_RX_BUFFER_SIZE/2)) {
            searchPos = (rxHead + UART_RX_BUFFER_SIZE - 100) % UART_RX_BUFFER_SIZE;
        }
        
        // Process GPS data at regular intervals
        uint32_t currentTime = HAL_GetTick();
        if (currentTime - lastGpsProcessTime >= GPS_PROCESS_INTERVAL) {
            lastGpsProcessTime = currentTime;
            
            // Process GPS data
            ProcessGpsData(); // temporarily remove to test dummy gps data
        }
        
        // Restart DMA reception
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uartRxBuffer, UART_RX_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); // Disable Half Transfer interrupt
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        // Check if we have received GPS data from the app (26 bytes)
        if (huart->RxXferSize == BT_GPS_DATA_SIZE) {
            // Store previous phone GPS data before updating
            memcpy(&previous_phone_gps_data, &phone_gps_data, sizeof(GPS_Data));
            
            // Process the received GPS data
            parsePhoneGPSData(bt_gps_buffer, &phone_gps_data);

            printToConsole("\r\nBluetooth GPS data: fix=%s lat=%.6f%c lon=%.6f%c\r\n", 
                phone_gps_data.fix_valid ? "VALID" : "INVALID",
                phone_gps_data.latitude, phone_gps_data.lat_direction,
                phone_gps_data.longitude, phone_gps_data.lon_direction);
            
        } else {
            // Original behavior for other data
            printToConsole("\r\nReceived (callback): %s", rx_buf);
        }
        
        // Restart the reception for GPS data
        memset(bt_gps_buffer, 0, BT_GPS_DATA_SIZE);

        HAL_UART_Receive_DMA(&huart1, bt_gps_buffer, BT_GPS_DATA_SIZE);
    }
}

// Set PWM duty cycle
void PWM_SetDutyCycle(TIM_HandleTypeDef *htim, uint32_t Channel, uint16_t dutyCycle) {
    uint16_t pulse = (__HAL_TIM_GET_AUTORELOAD(htim) * dutyCycle) / 100;
    __HAL_TIM_SET_COMPARE(htim, Channel, pulse);
}

// Scale function: maps 0-100 to 50-100
int scalePWM(int speed) {
    if (speed == 0) return 0;  // Keep 0 as 0 for complete stop
    int absSpeed = speed > 0 ? speed : -speed;
    return 50 + (absSpeed * 50) / 100;  // Maps 0-100 to 50-100
}

/**
 * Control both motors with a single function
 * @param leftSpeed: Speed for left motor (-100 to +100)
 *                  Positive values = forward, Negative values = reverse
 * @param rightSpeed: Speed for right motor (-100 to +100)
 *                   Positive values = forward, Negative values = reverse
 */
void controlMotors(int leftSpeed, int rightSpeed) {
    // Constrain speeds to valid range
    leftSpeed = (leftSpeed > 100) ? 100 : leftSpeed;
    leftSpeed = (leftSpeed < -100) ? -100 : leftSpeed;
    rightSpeed = (rightSpeed > 100) ? 100 : rightSpeed;
    rightSpeed = (rightSpeed < -100) ? -100 : rightSpeed;
    
    // Set left motor
    if (leftSpeed >= 0) {
        // Forward
        PWM_SetDutyCycle(&MOTOR_LEFT_FWD_TIM, MOTOR_LEFT_FWD_CHANNEL, scalePWM(leftSpeed));
        PWM_SetDutyCycle(&MOTOR_LEFT_REV_TIM, MOTOR_LEFT_REV_CHANNEL, 0);
    } else {
        // Reverse
        PWM_SetDutyCycle(&MOTOR_LEFT_FWD_TIM, MOTOR_LEFT_FWD_CHANNEL, 0);
        PWM_SetDutyCycle(&MOTOR_LEFT_REV_TIM, MOTOR_LEFT_REV_CHANNEL, scalePWM(-leftSpeed));
    }
    
    // Set right motor
    if (rightSpeed >= 0) {
        // Forward
        PWM_SetDutyCycle(&MOTOR_RIGHT_FWD_TIM, MOTOR_RIGHT_FWD_CHANNEL, scalePWM(rightSpeed));
        PWM_SetDutyCycle(&MOTOR_RIGHT_REV_TIM, MOTOR_RIGHT_REV_CHANNEL, 0);
    } else {
        // Reverse
        PWM_SetDutyCycle(&MOTOR_RIGHT_FWD_TIM, MOTOR_RIGHT_FWD_CHANNEL, 0);
        PWM_SetDutyCycle(&MOTOR_RIGHT_REV_TIM, MOTOR_RIGHT_REV_CHANNEL, scalePWM(-rightSpeed));
    }
}

// imu libraries
typedef struct {
    int16_t x_accel;
    int16_t y_accel;
    int16_t z_accel;
    int16_t x_gyro;
    int16_t y_gyro;
    int16_t z_gyro;
} icm_20948_data;

void activate_imu() {
    HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_RESET);
}

void deactivate_imu() {
    HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_SET);
}

uint8_t SPI_Read(uint8_t reg) {
    uint8_t rx_data = 0;
    uint8_t tx_data = reg | 0x80;  // Set the read bit (bit 7) to high
    activate_imu();    
    HAL_Delay(1);
    HAL_SPI_Transmit(&hspi5, &tx_data, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi5, &rx_data, 1, HAL_MAX_DELAY);
    HAL_Delay(1);
    deactivate_imu();
    return rx_data;
}

void SPI_Write(uint8_t reg, uint8_t data) {
    uint8_t tx_data[2];
    tx_data[0] = reg & 0x7F;  // Clear the read bit (bit 7) to low
    tx_data[1] = data;
    activate_imu();
    HAL_Delay(1);
    HAL_SPI_Transmit(&hspi5, tx_data, 2, HAL_MAX_DELAY);
    HAL_Delay(1);
    deactivate_imu();
}

// Helpers for magnetometer I2C communication
void ICM_i2c_Mag_Write(uint8_t reg, uint8_t value) {
    SPI_Write(USER_BANK_SEL, USER_BANK_3);  // Select user bank 3
    HAL_Delay(1);
    
    SPI_Write(0x03, 0x0C);  // Set I2C_SLV0_ADDR to write mode
    HAL_Delay(1);
    
    SPI_Write(0x04, reg);   // Set I2C_SLV0_REG to register address
    HAL_Delay(1);
    
    SPI_Write(0x06, value); // Set I2C_SLV0_DO with value to write
    HAL_Delay(1);
}

uint8_t ICM_i2c_Mag_Read(uint8_t reg) {
    uint8_t data;
    
    SPI_Write(USER_BANK_SEL, USER_BANK_3);  // Select user bank 3
    HAL_Delay(1);
    
    SPI_Write(0x03, 0x0C|0x80);  // Set to read mode
    HAL_Delay(1);
    
    SPI_Write(0x04, reg);  // Set register to read
    HAL_Delay(1);
    
    SPI_Write(0x06, 0xFF);  // Dummy write to trigger read
    HAL_Delay(1);
    
    // Return to user bank 0 to read the data
    SPI_Write(USER_BANK_SEL, USER_BANK_0);
    
    // Read the data from EXT_SLV_SENS_DATA register
    data = SPI_Read(0x3B);
    HAL_Delay(1);
    
    return data;
}

void ICM_InitMag() {
    // Configure AUX I2C Magnetometer (onboard ICM-20948)
    
    // Select bank 0 and enable I2C master
    SPI_Write(USER_BANK_SEL, USER_BANK_0);
    HAL_Delay(10);
    SPI_Write(0x0F, 0x30);  // INT Pin / Bypass Enable Configuration
    HAL_Delay(10);
    SPI_Write(0x03, 0x20);  // I2C_MST_EN
    HAL_Delay(10);
    
    // Configure I2C master in bank 3
    SPI_Write(USER_BANK_SEL, USER_BANK_3);
    HAL_Delay(10);
    SPI_Write(0x01, 0x4D);  // I2C Master mode and Speed 400 kHz
    HAL_Delay(10);
    SPI_Write(0x02, 0x01);  // I2C_SLV0_DLY enable
    HAL_Delay(10);
    SPI_Write(0x05, 0x81);  // Enable IIC and EXT_SENS_DATA == 1 Byte
    HAL_Delay(10);
    
    // Reset magnetometer
    ICM_i2c_Mag_Write(MAG_CNTL3, 0x01);
    HAL_Delay(100);  // Wait for reset to complete
    
    // Set to continuous measurement mode & 16-bit output
    ICM_i2c_Mag_Write(MAG_CNTL2, 0x08);  // Mode 4 (100 Hz)
    HAL_Delay(10);
}

void ICM_ReadMag(int16_t magn[3]) {
    uint8_t mag_buffer[10];
    
    // Read raw magnetometer data
    mag_buffer[0] = ICM_i2c_Mag_Read(MAG_ST1);  // Status 1
    
    // Only proceed if data is ready (Data Ready bit in ST1)
    if (mag_buffer[0] & 0x01) {
        mag_buffer[1] = ICM_i2c_Mag_Read(MAG_HXL);
        mag_buffer[2] = ICM_i2c_Mag_Read(MAG_HXH);
        magn[0] = mag_buffer[1] | (mag_buffer[2] << 8);
        
        mag_buffer[3] = ICM_i2c_Mag_Read(MAG_HYL);
        mag_buffer[4] = ICM_i2c_Mag_Read(MAG_HYH);
        magn[1] = mag_buffer[3] | (mag_buffer[4] << 8);
        
        mag_buffer[5] = ICM_i2c_Mag_Read(MAG_HZL);
        mag_buffer[6] = ICM_i2c_Mag_Read(MAG_HZH);
        magn[2] = mag_buffer[5] | (mag_buffer[6] << 8);
        
        // Trigger next measurement
        ICM_i2c_Mag_Write(MAG_CNTL2, 0x08);
    }
}

// Update the init_imu function to include magnetometer initialization
void init_imu(void) {
    // Reset the device first
    deactivate_imu();
    HAL_Delay(10);
    activate_imu();
    HAL_Delay(10);

    // Reset the device
    SPI_Write(PWR_MGMT_1, 0x80);  // Device reset
    HAL_Delay(100);  // Wait for reset to complete
    SPI_Write(PWR_MGMT_1, 0x01);  // Auto select best clock source
    HAL_Delay(10);
    
    // Verify device ID
    uint8_t whoami = SPI_Read(WHO_AM_I_REG);
    printToConsole("WHO_AM_I register value: 0x%02X (expected: 0x%02X)\r\n", whoami, WHO_AM_I_VAL);
    
    if (whoami == WHO_AM_I_VAL) {
        printToConsole("ICM-20948 found!\r\n");
        
        // Configure the device further
        SPI_Write(PWR_MGMT_2, 0x00);  // Enable accel and gyro
        HAL_Delay(10);
        
        // Select bank 2 for gyro config
        SPI_Write(USER_BANK_SEL, USER_BANK_2);
        HAL_Delay(10);
        
        // Configure gyro
        SPI_Write(GYRO_CONFIG_1, GYRO_RATE_250 | GYRO_LPF_17HZ);
        HAL_Delay(10);
        
        // Set accelerometer config
        SPI_Write(0x14, 0x00);  // 2g full scale
        HAL_Delay(10);
        
        // Return to bank 0
        SPI_Write(USER_BANK_SEL, USER_BANK_0);
        HAL_Delay(10);
        
        // Initialize magnetometer
        ICM_InitMag();
        
        printToConsole("ICM-20948 configured successfully with magnetometer!\r\n");
    } else {
        printToConsole("Error: Unknown device ID or communication failure!\r\n");
        printToConsole("Trying alternative initialization...\r\n");
        
        // Try a more robust initialization sequence
        HAL_Delay(100);
        
        // Make sure CS is high then low to reset SPI interface
        HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_SET);
        HAL_Delay(10);
        HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_RESET);
        HAL_Delay(10);
        HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_SET);
        HAL_Delay(10);
        
        // Try reading WHO_AM_I again
        whoami = SPI_Read(WHO_AM_I_REG);
        printToConsole("Second attempt WHO_AM_I: 0x%02X\r\n", whoami);
    }
}

/**
 * Calculate bearing using accelerometer and magnetometer data
 * @return Bearing in degrees (0-360°, where 0/360° is North)
 */
float calculateBearing(int16_t accel[3], int16_t mag[3]) {
    // Store new samples in history arrays
    for (int i = 0; i < 3; i++) {
        accel_history[history_index][i] = accel[i];
        mag_history[history_index][i] = mag[i];
    }
    
    // Move to next position in circular buffer
    history_index = (history_index + 1) % ROLLING_AVG_SAMPLES;
    
    // Calculate averages from history
    float avg_accel[3] = {0.0f};
    float avg_mag[3] = {0.0f};
    
    for (int i = 0; i < ROLLING_AVG_SAMPLES; i++) {
        for (int j = 0; j < 3; j++) {
            avg_accel[j] += accel_history[i][j] / (float)ROLLING_AVG_SAMPLES;
            avg_mag[j] += mag_history[i][j] / (float)ROLLING_AVG_SAMPLES;
        }
    }
    
    // Use averaged values for bearing calculation
    float ax = avg_accel[0];
    float ay = avg_accel[1];
    float az = avg_accel[2];
    
    float mx = avg_mag[0];
    float my = avg_mag[1];
    float mz = avg_mag[2];
    
    // Calculate pitch and roll from accelerometer
    // Note: This assumes standard orientation where:
    // +X points right, +Y points forward, +Z points up
    float pitch = atan2(ay, sqrt(ax*ax + az*az));
    float roll = atan2(-ax, az);
    
    // Tilt-compensate the magnetometer readings
    float mx_comp = mx * cos(pitch) + mz * sin(pitch);
    float my_comp = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);
    
    // Calculate heading in radians
    float heading = atan2(my_comp, mx_comp);
    
    // Convert to degrees
    heading = heading * 180.0f / M_PI;
    
    // Add declination correction
    heading += MAGNETIC_DECLINATION_DEG;
    
    // Normalize to 0-360 degrees
    if (heading < 0) {
        heading += 360.0f;
    } else if (heading >= 360.0f) {
        heading -= 360.0f;
    }
    
    return heading;
}

/**
 * Get a cardinal direction name from a bearing
 * @param bearing The bearing in degrees (0-360)
 * @return String with cardinal direction
 */
const char* getCardinalDirection(float bearing) {
    const char* directions[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
    int index = (int)round(bearing / 45.0f) % 8;
    return directions[index];
}

// Update the read_imu_data function to include bearing calculation
float read_imu_data(void) {
    int16_t accel[3];
    // int16_t gyro[3];
    
    // Read accelerometer data
    uint8_t accel_x_h = SPI_Read(ACCEL_XOUT_H);
    uint8_t accel_x_l = SPI_Read(ACCEL_XOUT_L);
    uint8_t accel_y_h = SPI_Read(ACCEL_YOUT_H);
    uint8_t accel_y_l = SPI_Read(ACCEL_YOUT_L);
    uint8_t accel_z_h = SPI_Read(ACCEL_ZOUT_H);
    uint8_t accel_z_l = SPI_Read(ACCEL_ZOUT_L);
    
    // Read gyroscope data
    // uint8_t gyro_x_h = SPI_Read(GYRO_XOUT_H);
    // uint8_t gyro_x_l = SPI_Read(GYRO_XOUT_L);
    // uint8_t gyro_y_h = SPI_Read(GYRO_YOUT_H);
    // uint8_t gyro_y_l = SPI_Read(GYRO_YOUT_L);
    // uint8_t gyro_z_h = SPI_Read(GYRO_ZOUT_H);
    // uint8_t gyro_z_l = SPI_Read(GYRO_ZOUT_L);
    
    // Combine high and low bytes
    accel[0] = (int16_t)((accel_x_h << 8) | accel_x_l);
    accel[1] = (int16_t)((accel_y_h << 8) | accel_y_l);
    accel[2] = (int16_t)((accel_z_h << 8) | accel_z_l);
    
    // gyro[0] = (int16_t)((gyro_x_h << 8) | gyro_x_l);
    // gyro[1] = (int16_t)((gyro_y_h << 8) | gyro_y_l);
    // gyro[2] = (int16_t)((gyro_z_h << 8) | gyro_z_l);
    
    // Read magnetometer data
    ICM_ReadMag(mag_data);
    
    // Calculate bearing
    float bearing = calculateBearing(accel, mag_data);
    // const char* direction = getCardinalDirection(bearing);
    
    // printToConsole("Bearing: %.1f° (%s)\r\n", bearing, direction);
    // Prints Bearing: 222.4° (SW)
    return bearing;
}


/**
 * Convert bytes to float considering potential endianness differences
 * @param bytes Pointer to 4 bytes of data
 * @return Converted float value
 */
float bytesToFloat(uint8_t* bytes) {
    union {
        float value;
        uint8_t b[4];
    } u;
    
    // Maintain byte order (React Native uses little-endian)
    memcpy(u.b, bytes, 4);
    return u.value;
}

/**
 * Parse binary GPS data received from phone app via Bluetooth
 * @param buffer 26-byte buffer containing GPS data
 * @param gps Pointer to GPS_Data structure to populate
 */
void parsePhoneGPSData(uint8_t* buffer, GPS_Data* gps) {
    // Validate checksum first
    uint8_t checksum = 0;
    for(int i = 0; i < 25; i++) checksum ^= buffer[i];
    
    if(checksum != buffer[25]) {
        printToConsole("Checksum failed: %02X vs %02X\r\n", checksum, buffer[25]);
        gps->fix_valid = false;
        return;
    }

    // Parse time (3 bytes)
    gps->hours = buffer[0];
    gps->minutes = buffer[1];
    gps->seconds = buffer[2];
    
    // Parse date (3 bytes + 2 byte year)
    gps->day = buffer[3];
    gps->month = buffer[4];
    gps->year = (buffer[5] | (buffer[6] << 8));  // Full 16-bit year
    
    // Parse coordinates (4 byte float + 1 byte direction each)
    gps->latitude = bytesToFloat(&buffer[7]);
    gps->lat_direction = buffer[11];
    gps->longitude = bytesToFloat(&buffer[12]);
    gps->lon_direction = buffer[16];
    
    // Parse movement (4 byte floats)
    gps->speed_knots = bytesToFloat(&buffer[17]);
    gps->course = bytesToFloat(&buffer[21]);
    
    // Validate directions and coordinates
    gps->fix_valid = ((gps->lat_direction == 'N' || gps->lat_direction == 'S') &&
                     (gps->lon_direction == 'E' || gps->lon_direction == 'W') &&
                     fabs(gps->latitude) <= 90.0f &&
                     fabs(gps->longitude) <= 180.0f);

    // Apply direction signs
    if(gps->lat_direction == 'S') gps->latitude = -gps->latitude;
    if(gps->lon_direction == 'W') gps->longitude = -gps->longitude;

    // Debug output
    printToConsole("Parsed Phone GPS: %.6f%c, %.6f%c Speed: %.2f Course: %.2f\r\n",
        gps->latitude, gps->lat_direction,
        gps->longitude, gps->lon_direction,
        gps->speed_knots, gps->course);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  printToConsole("Starting Program!\r\n");
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_SPI5_Init();
  /* USER CODE BEGIN 2 */

  init_imu();

    //   resetBluetoothModule();
    HAL_Delay(1000);
    //   initBluetooth();

  // Clear the Bluetooth GPS buffer before starting reception
  memset(bt_gps_buffer, 0, BT_GPS_DATA_SIZE);
  HAL_UART_Receive_DMA(&huart1, bt_gps_buffer, BT_GPS_DATA_SIZE);
  printToConsole("Bluetooth GPS reception initialized!\r\n");

  // Initialize DMA for UART6 reception
  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, uartRxBuffer, UART_RX_BUFFER_SIZE);
  __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT); // Disable Half Transfer interrupt
  printToConsole("DMA Initialized successfully\r\n");

  // Test USART6 reception
  // testUSART6Reception();

  // Start PWM channels
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  
  // Initialize motors to stopped state
  controlMotors(0, 0);
  
  // Optional: Print motor control initialization message
  printToConsole("Motor control initialized successfully\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Print the bearing from the IMU for debugging
    if (DEBUG_IMU_DATA) {
        float bearing = read_imu_data();
        printToConsole("Current Bearing: %.1f degrees\r\n", bearing);
    }
    
    // Check if GPS data has changed
    bool new_data_received = false;

    if (gps_data.fix_valid && 
        (gps_data.latitude != previous_gps_data.latitude ||
         gps_data.longitude != previous_gps_data.longitude ||
         gps_data.lat_direction != previous_gps_data.lat_direction ||
         gps_data.lon_direction != previous_gps_data.lon_direction)) {
        new_data_received = true;
        printToConsole("GPS data changed\r\n");
    }
    
    if (phone_gps_data.fix_valid && 
        (phone_gps_data.latitude != previous_phone_gps_data.latitude ||
         phone_gps_data.longitude != previous_phone_gps_data.longitude ||
         phone_gps_data.lat_direction != previous_phone_gps_data.lat_direction ||
         phone_gps_data.lon_direction != previous_phone_gps_data.lon_direction)) {
        new_data_received = true;
        printToConsole("Phone GPS data changed\r\n");
    }
    
    if (new_data_received) {
        printToConsole("\r\nNew data received!\r\n");
        printToConsole("gps_data.fix_valid: %d\r\n", gps_data.fix_valid);
        printToConsole("phone_gps_data.fix_valid: %d\r\n", phone_gps_data.fix_valid);
        // Calculate vector between GPS positions only if data has changed
        if (new_data_received && gps_data.fix_valid && phone_gps_data.fix_valid) {
            gnss_vector = calculateGNSSVector(gps_data, phone_gps_data);
            printToConsole("Recalculated GNSS vector\r\n");
            
            // Display vector information
            printToConsole("Distance between points: %.2f meters\r\n", gnss_vector.distance);
            printToConsole("Bearing between points: %.1f degrees\r\n", gnss_vector.bearing);
            
            // Get current bearing from IMU
            float bearing = read_imu_data();
            printToConsole("Current Bearing: %.1f degrees\r\n", bearing);
            
            // Calculate the difference between the two bearings
            float difference = bearing - gnss_vector.bearing;
            // Normalize the difference to -180 to 180 degrees
            if (difference > 180) difference -= 360;
            if (difference < -180) difference += 360;
            
            printToConsole("Difference between bearings: %.1f degrees\r\n", difference);
            
            printToConsole("Robot Should Turn %s by %.1f degrees\r\n", 
                difference > 0 ? "left" : "right", fabs(difference));

            const int MAX_SPEED = 100;
            const int MIN_SPEED = 20;
            const float MAX_DISTANCE = 10.0f;  // meters
            const float MIN_DISTANCE = 1.0f;   // meters
            const float MAX_ANGLE_DIFF = 90.0f;
            const float MIN_ANGLE_DIFF = 2.0f;
            
            // Calculate base forward speed based on distance
            float distanceSpeed = 0;
            if (gnss_vector.distance > MAX_DISTANCE) {
                distanceSpeed = MAX_SPEED;
            } else if (gnss_vector.distance < MIN_DISTANCE) {
                distanceSpeed = MIN_SPEED;
            } else {
                // More aggressive speed curve using square root for better low-speed control
                float normalizedDistance = (gnss_vector.distance - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE);
                distanceSpeed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * sqrtf(normalizedDistance);
            }
            
            // Calculate turn intensity based on bearing difference
            float turnIntensity = 0;
            float absDifference = fabs(difference);
            if (absDifference < MIN_ANGLE_DIFF) {
                // Almost aligned, very minimal turning
                turnIntensity = 0.05f;  // Reduced from 0.1f for straighter forward motion
            } else if (absDifference > MAX_ANGLE_DIFF) {
                // Maximum turning - make it more aggressive
                turnIntensity = 1.0f;
            } else {
                // More aggressive turn response curve
                turnIntensity = 0.05f + 0.95f * powf((absDifference - MIN_ANGLE_DIFF) / 
                                                  (MAX_ANGLE_DIFF - MIN_ANGLE_DIFF), 0.7f);
            }
            
            // Calculate left and right motor speeds
            int leftSpeed = 0;
            int rightSpeed = 0;
            
            if (absDifference < MIN_ANGLE_DIFF) {
                // Almost aligned, go straight at full calculated speed
                leftSpeed = rightSpeed = (int)distanceSpeed;
            } else {
                // Need to turn
                float turnReduction = turnIntensity * 0.8f;  // Reduce turn intensity effect
                
                if (absDifference > 45.0f) {  // If angle difference is large, do a point turn
                    int turnSpeed = (int)(MAX_SPEED * 0.7f);  // Use 70% of max speed for turning
                    if (difference > 0) {
                        // Turn left in place
                        leftSpeed = -turnSpeed;
                        rightSpeed = turnSpeed;
                    } else {
                        // Turn right in place
                        leftSpeed = turnSpeed;
                        rightSpeed = -turnSpeed;
                    }
                } else {
                    // Normal turning behavior with forward motion
                    if (difference > 0) {
                        // Turn left while moving forward
                        rightSpeed = (int)distanceSpeed;
                        leftSpeed = (int)(distanceSpeed * (1.0f - turnReduction));
                    } else {
                        // Turn right while moving forward
                        leftSpeed = (int)distanceSpeed;
                        rightSpeed = (int)(distanceSpeed * (1.0f - turnReduction));
                    }
                    
                    // Ensure minimum forward motion for small turns
                    int minTurnSpeed = (int)(distanceSpeed * 0.3f);
                    leftSpeed = fmax(leftSpeed, minTurnSpeed);
                    rightSpeed = fmax(rightSpeed, minTurnSpeed);
                }
            }
            
            // Apply motor speeds
            printToConsole("Motor speeds: Left=%d, Right=%d (Distance: %.2fm, Turn: %.2f, Angle: %.1f)\r\n", 
                        leftSpeed, rightSpeed, gnss_vector.distance, turnIntensity, absDifference);
            controlMotors(leftSpeed, rightSpeed);

            // Update previous data
            memcpy(&previous_gps_data, &gps_data, sizeof(GPS_Data));
            memcpy(&previous_phone_gps_data, &phone_gps_data, sizeof(GPS_Data));
        } else {
            printToConsole("Missing valid GPS fixes: Device %s, Phone %s\r\n",
                        gps_data.fix_valid ? "OK" : "BAD",
                        phone_gps_data.fix_valid ? "OK" : "BAD");
            controlMotors(0, 0); // Stop motors if invalid data
        }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi5.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 200;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 200;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
