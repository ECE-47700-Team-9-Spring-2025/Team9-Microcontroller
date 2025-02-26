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
#include "string.h"
#include <stdarg.h>
#include <stdio.h>
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "m8q.h"
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
#define ICM_CS_PIN       GPIO_PIN_15
#define ICM_CS_PORT      GPIOA

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
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2004c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2004c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2004c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2004c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
void printToConsole(const char *format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    strcat(buffer, "\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart5) {
        // Log the received data
        printToConsole("Received (callback): %s", rx_buf);
        
        // Restart the reception for the next data
        HAL_UART_Receive_DMA(&huart5, rx_buf, sizeof(rx_buf));
    }
}

// Update SPI read/write functions
uint8_t SPI_Read(uint8_t reg) {
    uint8_t rx_data = 0;
    uint8_t tx_data = reg | 0x80;  // Set the read bit (bit 7)
    
    HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_RESET);  // CS LOW
    
    // Small delay to ensure CS is stable before transmission
    for(volatile int i = 0; i < 10; i++);
    
    HAL_SPI_Transmit(&hspi1, &tx_data, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &rx_data, 1, HAL_MAX_DELAY);
    
    // Small delay before raising CS
    for(volatile int i = 0; i < 10; i++);
    
    HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_SET);    // CS HIGH
    
    return rx_data;
}

void SPI_Write(uint8_t reg, uint8_t data) {
    uint8_t tx_data[2];
    tx_data[0] = reg & 0x7F;  // Clear the read bit (bit 7)
    tx_data[1] = data;
    
    HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_RESET);  // CS LOW
    
    // Small delay to ensure CS is stable before transmission
    for(volatile int i = 0; i < 10; i++);
    
    HAL_SPI_Transmit(&hspi1, tx_data, 2, HAL_MAX_DELAY);
    
    // Small delay before raising CS
    for(volatile int i = 0; i < 10; i++);
    
    HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_SET);    // CS HIGH
}

// Update initialization sequence
void init_imu(void) {
    // Configure CS pin as output and set it high initially
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Enable clock for CS pin port if not already enabled
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    // Configure CS pin as output
    GPIO_InitStruct.Pin = ICM_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ICM_CS_PORT, &GPIO_InitStruct);
    
    // Set CS high initially
    HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_SET);
    HAL_Delay(100);  // Give the sensor some time to power up
    
    // Reset the device first
    SPI_Write(PWR_MGMT_1, 0x80);  // Device reset
    HAL_Delay(100);  // Wait for reset to complete
    
    // Wake up the device
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
    printToConsole("Accel: X=%d, Y=%d, Z=%d", accel_x, accel_y, accel_z);
    printToConsole("Gyro: X=%d, Y=%d, Z=%d", gyro_x, gyro_y, gyro_z);
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ETH_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART5_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
    
    while (idx < (maxSize - 1)) {
        if ((HAL_GetTick() - startTick) > 2000) {
            printToConsole("Timeout after %lu ms\r\n", (HAL_GetTick() - startTick));
            break;
        }
        
        HAL_StatusTypeDef status = HAL_UART_Receive(&huart6, (uint8_t*)&ch, 1, 100);
        if (status == HAL_OK) {
            if (ch == '$') {
                startFound = true;
                idx = 0;
            }
            
            if (startFound) {
                buffer[idx++] = ch;
                if (ch == '\n' && idx > 1 && buffer[idx-2] == '\r') {
                    buffer[idx] = '\0';
                    return true;
                }
            }
        }
    }
    
    buffer[idx] = '\0';
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
        
        const char* buffer = "$GNRMC,201850.00,A,4025.69979,N,08654.69118,W,0.256,,240225,08654.69118,W,0.256,,240225,,,A*73";
        
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
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_ETH_Init();
  MX_I2C1_Init();
  MX_UART5_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  int size = strlen(tx_1);
  int size2 = strlen(tx_2);
  init_imu();


  // HAL_UART_Receive_DMA(&huart5, rx_buf, size);
  // HAL_UART_Transmit_DMA(&huart5, (uint8_t*)tx_1, size);
  // printToConsole("Sent: %s", tx_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Print the GPS output
    if (true) {
      // Try multiple times to get a valid sentence
      bool success = false;
      printToConsole("\r\n=== New GPS Reading ===\r\n");
      for(int attempts = 0; attempts < 3 && !success; attempts++) {
          if (attempts > 0) {
              printToConsole("Retry attempt %d\r\n", attempts);
          }
          success = printCurrentGpsOutput();
      }
      if (!success) {
          printToConsole("Failed after all attempts\r\n");
      }
    }

    // Wait for 500ms before triggering again
    HAL_Delay(1000);
    read_imu_data();
    HAL_Delay(1000);
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00808CD2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;  // Changed to software NSS control
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;  // Reduced speed for reliability
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

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
