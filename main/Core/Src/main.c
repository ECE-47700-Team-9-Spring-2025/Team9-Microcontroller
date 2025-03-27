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
static char* tx_2 = "Hello World";

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
#define ICM_CS_PORT      GPIOC


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

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */
GPS_Data gps_data;

// DMA buffer for UART reception
#define UART_RX_BUFFER_SIZE 512
uint8_t uartRxBuffer[UART_RX_BUFFER_SIZE];
volatile uint16_t rxHead = 0;
volatile uint16_t searchPos = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

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

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART6) {
        // Debug output to confirm callback is working
        // printToConsole("DMA received %d bytes\r\n", Size);
        
        // Calculate the new head position
        uint16_t newHead = (rxHead + Size) % UART_RX_BUFFER_SIZE;
        rxHead = newHead;
        
        // If searchPos is far behind, move it forward to avoid searching old data
        if ((rxHead > searchPos && (rxHead - searchPos) > UART_RX_BUFFER_SIZE/2) ||
            (rxHead < searchPos && (UART_RX_BUFFER_SIZE - searchPos + rxHead) > UART_RX_BUFFER_SIZE/2)) {
            searchPos = (rxHead + UART_RX_BUFFER_SIZE - 100) % UART_RX_BUFFER_SIZE;
            // printToConsole("Adjusted searchPos to %d\r\n", searchPos);
        }
        
        // Restart DMA reception
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uartRxBuffer, UART_RX_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); // Disable Half Transfer interrupt
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1) {
        // Log the received data
        printToConsole("Received (callback): %s", rx_buf);
        
        // Restart the reception for the next data
        HAL_UART_Receive_DMA(&huart1, rx_buf, sizeof(rx_buf));
    }
}

// Set PWM duty cycle
// void PWM_SetDutyCycle(TIM_HandleTypeDef *htim, uint32_t Channel, uint16_t dutyCycle) {
//     uint16_t pulse = (__HAL_TIM_GET_AUTORELOAD(htim) * dutyCycle) / 100;
//     __HAL_TIM_SET_COMPARE(htim, Channel, pulse);
// }

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
    HAL_Delay(10);
    HAL_SPI_Transmit(&hspi2, &tx_data, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi2, &rx_data, 1, HAL_MAX_DELAY);
    HAL_Delay(10);
    deactivate_imu();
    return rx_data;
}

void SPI_Write(uint8_t reg, uint8_t data) {
    uint8_t tx_data[2];
    tx_data[0] = reg & 0x7F;  // Clear the read bit (bit 7) to low
    tx_data[1] = data;
    activate_imu();
    HAL_Delay(10);
    HAL_SPI_Transmit(&hspi2, tx_data, 2, HAL_MAX_DELAY);
    HAL_Delay(10);
    deactivate_imu();
}

// Update initialization sequence
void init_imu(void) {
    // Reset the device first
    deactivate_imu();
    HAL_Delay(10);
    activate_imu();
    HAL_Delay(10);

    // Reset the device
    SPI_Write(PWR_MGMT_1, 0x80);  // Device reset
    HAL_Delay(10);  // Wait for reset to complete
    SPI_Write(PWR_MGMT_1, 0x01);  // Auto select best clock source
    HAL_Delay(10);
    
    // Verify device ID
    uint8_t whoami = SPI_Read(WHO_AM_I_REG);
    printToConsole("WHO_AM_I register value: 0x%02X (expected: 0x%02X)", whoami, WHO_AM_I_VAL);
    
    if (whoami == WHO_AM_I_VAL) {
        printToConsole("ICM-20948 found!");
        
        // Configure the device further
        SPI_Write(PWR_MGMT_2, 0x00);  // Enable accel and gyro
        HAL_Delay(10);
        
        // Configure gyro
        SPI_Write(GYRO_CONFIG_1, 0x00);  // 250 dps full scale
        HAL_Delay(10);
        
        // Configure accelerometer
        SPI_Write(ACCEL_CONFIG, 0x00);  // 2g full scale
        HAL_Delay(10);
        
        printToConsole("ICM-20948 configured successfully!");
    } else {
        printToConsole("Error: Unknown device ID or communication failure!");
        printToConsole("Trying alternative initialization...");
        
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
        printToConsole("Second attempt WHO_AM_I: 0x%02X", whoami);
    }
}

void read_imu_data(void) {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    
    // Read accelerometer data
    uint8_t accel_x_h = SPI_Read(ACCEL_XOUT_H);
    uint8_t accel_x_l = SPI_Read(ACCEL_XOUT_L);
    uint8_t accel_y_h = SPI_Read(ACCEL_YOUT_H);
    uint8_t accel_y_l = SPI_Read(ACCEL_YOUT_L);
    uint8_t accel_z_h = SPI_Read(ACCEL_ZOUT_H);
    uint8_t accel_z_l = SPI_Read(ACCEL_ZOUT_L);
    
    // Read gyroscope data
    uint8_t gyro_x_h = SPI_Read(GYRO_XOUT_H);
    uint8_t gyro_x_l = SPI_Read(GYRO_XOUT_L);
    uint8_t gyro_y_h = SPI_Read(GYRO_YOUT_H);
    uint8_t gyro_y_l = SPI_Read(GYRO_YOUT_L);
    uint8_t gyro_z_h = SPI_Read(GYRO_ZOUT_H);
    uint8_t gyro_z_l = SPI_Read(GYRO_ZOUT_L);
    
    // Combine high and low bytes
    accel_x = (int16_t)((accel_x_h << 8) | accel_x_l);
    accel_y = (int16_t)((accel_y_h << 8) | accel_y_l);
    accel_z = (int16_t)((accel_z_h << 8) | accel_z_l);
    
    gyro_x = (int16_t)((gyro_x_h << 8) | gyro_x_l);
    gyro_y = (int16_t)((gyro_y_h << 8) | gyro_y_l);
    gyro_z = (int16_t)((gyro_z_h << 8) | gyro_z_l);
    
    // Print the data
    printToConsole("Accel: X=%d, Y=%d, Z=%d\r\n", accel_x, accel_y, accel_z);
    printToConsole("Gyro: X=%d, Y=%d, Z=%d\r\n", gyro_x, gyro_y, gyro_z);
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
  /* USER CODE BEGIN 2 */

  int size = strlen(tx_1);
  init_imu();
  HAL_UART_Receive_DMA(&huart1, rx_buf, size);
  HAL_UART_Transmit_DMA(&huart1, (uint8_t*)tx_1, size);
  printToConsole("Sent: %s", tx_1);

  // Initialize DMA for UART6 reception
  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, uartRxBuffer, UART_RX_BUFFER_SIZE);
  __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT); // Disable Half Transfer interrupt
  printToConsole("DMA Initialized!\r\n");

  // Variables for test cycling
  typedef enum {
    TEST_IMU,
    TEST_GPS,
    TEST_BLUETOOTH,
    TEST_COUNT
  } TestMode;
  
  TestMode currentTest = TEST_IMU;
  uint32_t testStartTime = 0;
  const uint32_t TEST_DURATION_MS = 15000; // 6 seconds per test
  
  // Initialize test start time
  testStartTime = HAL_GetTick();
  printToConsole("\r\n=== Starting IMU Test ===\r\n");

  // Test USART6 reception
  // testUSART6Reception();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Check if it's time to switch tests
    uint32_t currentTime = HAL_GetTick();
    if (currentTime - testStartTime >= TEST_DURATION_MS) {
      // Switch to next test
      currentTest = (currentTest + 1) % TEST_COUNT;
      testStartTime = currentTime;
      
      // Print header for new test
      switch (currentTest) {
        case TEST_IMU:
          printToConsole("\r\n=== Starting IMU Test ===\r\n");
          break;
        case TEST_GPS:
          printToConsole("\r\n=== Starting GPS Test ===\r\n");
          break;
        case TEST_BLUETOOTH:
          printToConsole("\r\n=== Starting Bluetooth Test ===\r\n");
          break;
      }
    }
    
    // Run the current test
    switch (currentTest) {
      case TEST_IMU:
        // reg bank select, page 54
        HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_RESET);
        uint8_t reg = 0x7F;
        uint8_t data = 0;
        HAL_SPI_Transmit(&hspi2, &reg, 1, 100);
        HAL_SPI_Transmit(&hspi2, &data, 1, 100);
        HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_SET);
        HAL_Delay(100);

        // WHO AM I REGISTER, PAGE 36
        HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_RESET);
        reg = 0x00 | 0x80;
        HAL_SPI_Transmit(&hspi2, &reg, 1, 100);
        HAL_SPI_Receive(&hspi2, &data, 1, 100);
        printToConsole("WHO AM I: %02X\r\n", data);
        HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_SET);
        HAL_Delay(100);

        // ICM READ DATA, PAGE 42
        // HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_RESET);
        // uint8_t data_rx[12];
        // uint8_t temp_data = 0x80 | ACCEL_XOUT_H;
        // HAL_SPI_Transmit(&hspi2, &temp_data, 1, 1000);
        // HAL_SPI_Receive(&hspi2, data_rx, 12, 1000);

        // uint16_t accel_x = ((int16_t)data_rx[0]<<8) | data_rx[1];
        // uint16_t accel_y = ((int16_t)data_rx[2]<<8) | data_rx[3];
        // uint16_t accel_z = ((int16_t)data_rx[4]<<8) | data_rx[5];
        // printToConsole("ACCEL XOUT: %04X\r\n", accel_x);
        // printToConsole("ACCEL YOUT: %04X\r\n", accel_y);
        // printToConsole("ACCEL ZOUT: %04X\r\n", accel_z);

        // HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_SET);
        // HAL_Delay(100);
        read_imu_data();
        
        break;
        
      case TEST_GPS:
        // Calculate the vector between the two GPS points
        GPS_Data gps_data1 = {
            .latitude = 37.78,
            .lat_direction = 'N',
            .longitude = -122.42,
            .lon_direction = 'W',
            .speed_knots = 10.0,
            .course = 270.0,
            .fix_valid = true
        };
        GPS_Data gps_data2 = {
            .latitude = 37.77,
            .lat_direction = 'N',
            .longitude = -122.43,
            .lon_direction = 'W',
            .speed_knots = 10.0,
            .course = 270.0,
            .fix_valid = true
        };
        GNSSVector gnss_vector = calculateGNSSVector(gps_data1, gps_data2);
        printToConsole("Distance: %.2f meters\r\n", gnss_vector.distance);
        printToConsole("Bearing: %.1f degrees\r\n", gnss_vector.bearing);
        printToConsole("Vector (N,E): (%.2f, %.2f)\r\n", gnss_vector.vector_north, gnss_vector.vector_east);
        break;
        
        // GPS test code
        char nmeaBuffer[256];
        if (getNMEASentence(nmeaBuffer, sizeof(nmeaBuffer))) {
          // Debug raw NMEA sentence
          printToConsole("\r\n--- Raw NMEA Sentence ---\r\n");
          printToConsole("Length: %d bytes\r\n", strlen(nmeaBuffer));
          printToConsole("Content: %s", nmeaBuffer);
          
          if (strstr(nmeaBuffer, "$GNRMC")) {
            printToConsole("\r\n=== GNRMC Message Detected ===\r\n");
            
            // Debug each field before parsing
            char *saveptr;
            char *token = strtok_r(nmeaBuffer, ",", &saveptr);
            int fieldIndex = 0;
            
            while (token != NULL) {
              switch(fieldIndex) {
                case 0: printToConsole("Message ID: %s\r\n", token); break;
                case 1: printToConsole("UTC Time: %s\r\n", token); break;
                case 2: printToConsole("Status: %s (%s)\r\n", token, 
                        (token[0] == 'A') ? "Active" : "Void"); break;
                case 3: printToConsole("Latitude: %s\r\n", token); break;
                case 4: printToConsole("N/S Indicator: %s\r\n", token); break;
                case 5: printToConsole("Longitude: %s\r\n", token); break;
                case 6: printToConsole("E/W Indicator: %s\r\n", token); break;
                case 7: printToConsole("Speed (knots): %s\r\n", token); break;
                case 8: printToConsole("Course: %s\r\n", token); break;
                case 9: printToConsole("Date: %s\r\n", token); break;
                default: printToConsole("Field %d: %s\r\n", fieldIndex, token);
              }
              token = strtok_r(NULL, ",", &saveptr);
              fieldIndex++;
            }
            
            printToConsole("Total fields: %d (expecting 12-13)\r\n", fieldIndex);
            
            // Try to parse with M8Q_ParseGNRMC
            if (M8Q_ParseGNRMC(nmeaBuffer, &gps_data)) {
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
            } else {
              printToConsole("\r\nParsing Failed!\r\n");
              printToConsole("Checksum validation: %s\r\n", 
                  (strchr(nmeaBuffer, '*') != NULL) ? "Present" : "Missing");
            }
            printToConsole("=========================\r\n");
          } 
          else if (strstr(nmeaBuffer, "$GNGGA")) {
            // ... existing GNGGA parsing code ...
          }
          else if (strstr(nmeaBuffer, "$GNTXT")) {
            // ... existing GNTXT parsing code ...
          }
          else {
            printToConsole("Other Message Type: %.5s\r\n", nmeaBuffer);
          }
        }
        
        // Small delay to prevent console flooding
        HAL_Delay(10);
        break;
        
      case TEST_BLUETOOTH:
        // Bluetooth test code
        static uint32_t lastBluetoothMsg = 0;
        
        // Send a test message every second
        if (currentTime - lastBluetoothMsg >= 1000) {
          lastBluetoothMsg = currentTime;
          
          // Send test message via UART1 (Bluetooth)
          char btMsg[64];
          snprintf(btMsg, sizeof(btMsg), "Bluetooth Test: %lu ms\r\n", currentTime);
          HAL_UART_Transmit(&huart1, (uint8_t*)btMsg, strlen(btMsg), HAL_MAX_DELAY);
          
          printToConsole("Sent to Bluetooth: %s", btMsg);
          
          // Check for any received data
          if (HAL_UART_Receive(&huart1, rx_buf, sizeof(rx_buf)-1, 10) == HAL_OK) {
            rx_buf[sizeof(rx_buf)-1] = '\0'; // Ensure null termination
            printToConsole("Received from Bluetooth: %s\r\n", rx_buf);
          }
        }
        
        HAL_Delay(100); // Small delay
        break;
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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
