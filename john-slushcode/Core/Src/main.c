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
#include "sr04.h"
#include "m8q.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
sr04_t sr04;
sr04_t sr04_tim4;
GPS_Data gps_data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void printToConsole(const char *format, ...) {
    char buffer[100]; // Adjust the buffer size as needed.
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

void printCurrentGpsOutput(void) {
    char buffer[128];
    uint32_t idx = 0;
    char ch;
    uint32_t startTick = HAL_GetTick();
    const uint32_t TOTAL_TIMEOUT = 1000; // 1000ms total timeout

    // Clear the buffer and GPS data
    memset(buffer, 0, sizeof(buffer));
    memset(&gps_data, 0, sizeof(GPS_Data));

    // Read until we get a complete sentence or timeout
    while (idx < (sizeof(buffer) - 1)) {
        // Check if we've exceeded total timeout
        if ((HAL_GetTick() - startTick) > TOTAL_TIMEOUT) {
            printToConsole("GPS: Timeout after %lu ms\r\n", (HAL_GetTick() - startTick));
            break;
        }

        HAL_StatusTypeDef status = HAL_UART_Receive(&huart6, (uint8_t *)&ch, 1, 10);
        if (status == HAL_OK) {
            // Print the character code for debugging
            // printToConsole("Received char: '%c' (0x%02X)\r\n", 
            //     (ch >= 32 && ch <= 126) ? ch : '.', 
            //     (unsigned char)ch);

            buffer[idx++] = ch;
        }
    }

    // For debugging: Use test buffer instead of received data
    const char *test_buffer = "$GPGGA,092750.000,5321.6802,N,00630.3372,W,1,8,1.03,61.7,M,55.2,M,,*76";
    strncpy(buffer, test_buffer, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';
    idx = strlen(buffer);

    if (idx > 0) {
        // Print raw sentence for debugging
        printToConsole("Raw GPS (%d bytes): %s\r\n", idx, buffer);

        // Check for different sentence types
        if (strstr(buffer, "$GPRMC") || strstr(buffer, "$GNRMC")) {
            if (M8Q_ParseGPRMC(buffer, &gps_data)) {
                printToConsole("\r\n----- GPS Data -----\r\n");
                
                // Print time
                printToConsole("Time: %02d:%02d:%02d\r\n", 
                    gps_data.hours, 
                    gps_data.minutes, 
                    gps_data.seconds);
                
                // Print date
                printToConsole("Date: %02d/%02d/%04d\r\n", 
                    gps_data.day, 
                    gps_data.month, 
                    gps_data.year);
                
                // Print position if fix is valid
                if (gps_data.fix_valid) {
                    printToConsole("Position: %.6f %c, %.6f %c\r\n",
                        gps_data.latitude, gps_data.lat_direction,
                        gps_data.longitude, gps_data.lon_direction);
                        
                    printToConsole("Speed: %.1f knots\r\n", 
                        gps_data.speed_knots);
                        
                    printToConsole("Course: %.1f degrees\r\n", 
                        gps_data.course);
                } else {
                    printToConsole("Position: No Fix\r\n");
                }
                
                printToConsole("-------------------\r\n");
            }
        } else if (strstr(buffer, "$GPGGA") || strstr(buffer, "$GNGGA")) {
            printToConsole("Parsing GGA message: %s\r\n", buffer);
            
            // Create temporary variables for parsing
            char *saveptr;
            char *token = strtok_r(buffer, ",", &saveptr);
            int field = 0;
            double latitude = 0.0, longitude = 0.0, altitude = 0.0;  // Changed to double
            char lat_dir = 0, lon_dir = 0;
            
            // Use strtok_r for safer tokenization
            while (token != NULL) {
                printToConsole("Field %d: %s\r\n", field, token);  // Debug print
                
                switch(field) {
                    case 2: // Latitude
                        latitude = atof(token);  // Changed to atof
                        printToConsole("Raw latitude: %.4f\r\n", latitude);
                        break;
                    case 3: // N/S indicator
                        lat_dir = token[0];
                        break;
                    case 4: // Longitude
                        longitude = atof(token);  // Changed to atof
                        printToConsole("Raw longitude: %.4f\r\n", longitude);
                        break;
                    case 5: // E/W indicator
                        lon_dir = token[0];
                        break;
                    case 9: // Altitude
                        altitude = atof(token);  // Changed to atof
                        printToConsole("Raw altitude: %.1f\r\n", altitude);
                        break;
                }
                field++;
                token = strtok_r(NULL, ",", &saveptr);
            }
            
            // Convert DDMM.MMMM to decimal degrees
            if (latitude != 0.0) {  // Only convert if we got a valid number
                double lat_degrees = (int)(latitude / 100);
                double lat_minutes = latitude - (lat_degrees * 100);
                latitude = lat_degrees + (lat_minutes / 60);
                if (lat_dir == 'S') latitude = -latitude;
                printToConsole("Converted latitude: %.6f\r\n", latitude);
            }
            
            if (longitude != 0.0) {  // Only convert if we got a valid number
                double lon_degrees = (int)(longitude / 100);
                double lon_minutes = longitude - (lon_degrees * 100);
                longitude = lon_degrees + (lon_minutes / 60);
                if (lon_dir == 'W') longitude = -longitude;
                printToConsole("Converted longitude: %.6f\r\n", longitude);
            }
            
            printToConsole("\r\n----- GGA Data -----\r\n");
            printToConsole("Latitude: %.6f° %c\r\n", latitude, lat_dir);
            printToConsole("Longitude: %.6f° %c\r\n", longitude, lon_dir);
            printToConsole("Altitude: %.1f m\r\n", altitude);
            printToConsole("-------------------\r\n");
        } else if (strstr(buffer, "$GNTXT")) {
            printToConsole("GPS Info Message: %s\r\n", buffer);
        } else {
            // For other sentence types, use generic decoder
            decodeNMEASentence(buffer);
        }
    } else {
        printToConsole("GPS: No data\r\n");
    }
}

void sendUBXCommand(const char* cmd, size_t len) {
    HAL_UART_Transmit(&huart6, (uint8_t*)cmd, len, 1000);
    HAL_Delay(100); // Wait for command to process
}

void initGPS(void) {
    printToConsole("Initializing GPS...\r\n");
    
    // First, flush any pending data
    HAL_Delay(100);
    uint8_t dummy;
    while (HAL_UART_Receive(&huart6, &dummy, 1, 1) == HAL_OK);
    
    // Reset to factory defaults
    const char reset[] = "$PUBX,41,1,0007,0003,9600,0*10\r\n";
    sendUBXCommand(reset, strlen(reset));
    HAL_Delay(1000);  // Give it time to reset
    
    // Disable all NMEA messages first
    const char disableAll[] = "$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n"
                             "$PUBX,40,GSA,0,0,0,0,0,0*4E\r\n"
                             "$PUBX,40,GSV,0,0,0,0,0,0*59\r\n"
                             "$PUBX,40,VTG,0,0,0,0,0,0*5E\r\n";
    sendUBXCommand(disableAll, strlen(disableAll));
    HAL_Delay(500);
    
    // Enable only the messages we want
    const char enableRMC[] = "$PUBX,40,RMC,0,1,0,0,0,0*46\r\n";
    sendUBXCommand(enableRMC, strlen(enableRMC));
    HAL_Delay(100);
    
    const char enableGGA[] = "$PUBX,40,GGA,0,1,0,0,0,0*5B\r\n";
    sendUBXCommand(enableGGA, strlen(enableGGA));
    HAL_Delay(100);
    
    printToConsole("GPS Initialization complete\r\n");
    HAL_Delay(1000);  // Give it time to start sending data
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
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  // Initialize GPS
  initGPS();

  // Print that we have to wait for GPS to get a fix
  printToConsole("Waiting for GPS to acquire fix (this may take a few minutes)...\r\n");
  HAL_Delay(5000);
  
  sr04.trig_port = GPIOA;
  sr04.trig_pin = GPIO_PIN_9;
  sr04.echo_htim = &htim1;
  sr04.echo_channel = TIM_CHANNEL_1;
  sr04_init(&sr04);

  // Trying to get two sensors working
  sr04_tim4.trig_port = GPIOB;
  sr04_tim4.trig_pin = GPIO_PIN_5;
  sr04_tim4.echo_htim = &htim4;
  sr04_tim4.echo_channel = TIM_CHANNEL_4;
  sr04_init(&sr04_tim4);

  // // Print that we have to wait 15 seconds before starting for gps to calibrate
  // printToConsole("Waiting for GPS to calibrate...\r\n");
  // HAL_Delay(25000);
  // printToConsole("GPS calibrated\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    printToConsole("--------------------------------\r\n\n");

    sr04_trigger(&sr04);
    sr04_trigger(&sr04_tim4);

    // Print the distance using UART
    printToConsole("Distance (Sensor 1): %lu mm\r\n", sr04.distance); // @suppress("Float formatting support")
    printToConsole("Distance (Sensor 2): %lu mm\r\n", sr04_tim4.distance); // @suppress("Float formatting support")
    
    // Print the GPS output from usart 6
    printCurrentGpsOutput();

    // Wait for 500ms before triggering again
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA9 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
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
