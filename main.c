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
#include <string.h> // Required for strlen
#include <stdio.h>  // Required for sprintf
#include <stdint.h> // Required for uint16_t, uint32_t
#include <stdlib.h> // Required for atoi
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Single byte to store UART input (for general UART testing, not directly related to potentiometer) */
uint8_t byte;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;      // For Potentiometer
TIM_HandleTypeDef htim2;      // For Servo Motor and new PWM sweep
TIM_HandleTypeDef htim3;      // For DC Motor
TIM_HandleTypeDef htim5;      // For Wall Clock Timer
UART_HandleTypeDef huart2;    // Main UART for printf
UART_HandleTypeDef huart6;    // Second UART for general UART test and Potentiometer output

/* USER CODE BEGIN PV */
// === VINEYARD IRRIGATION CONTROLLER GLOBALS ===

// System Mode
typedef enum {
    MODE_SETUP,
    MODE_RUN
} SystemMode_t;

// Pipeline Selection
typedef enum {
    PIPELINE_INLET = 0,
    PIPELINE_ZONE1 = 1,
    PIPELINE_ZONE2 = 2,
    PIPELINE_ZONE3 = 3
} Pipeline_t;

// Configuration Structure
typedef struct {
    uint8_t inlet_pwm_option;      // 0=Manual, 1-3=Fixed PWM
    uint8_t zone_order[3];         // Order of zones (1-3)
    uint8_t zone_pwm_option[3];    // PWM options for each zone
    uint8_t inlet_start_hour;      // Inlet operation start hour
    uint8_t inlet_stop_hour;       // Inlet operation stop hour
    uint8_t zone_start_hour[3];    // Zone operation start hours
    uint8_t zone_stop_hour[3];     // Zone operation stop hours
} IrrigationConfig_t;

// System State Variables
SystemMode_t system_mode = MODE_SETUP;
IrrigationConfig_t config;
volatile uint8_t config_complete = 0;

// Scaled Wall Clock (300x faster: 24h = 4.8min, 1h = 12s)
volatile uint8_t scaled_hours = 0;
volatile uint8_t scaled_mins = 0;
volatile uint8_t scaled_secs = 0;
volatile uint8_t wall_clock_update_flag = 0;

// Current Operation State
Pipeline_t current_pipeline = PIPELINE_INLET;
uint8_t motor_running = 0;
uint8_t motor_direction = 0;  // 0=Forward, 1=Reverse
uint8_t motor_pwm_percent = 0;

// Sensor Values
volatile uint16_t rpm_counter = 0;
volatile uint8_t rpm_tick_flag = 0;
uint16_t current_rpm = 0;
uint8_t water_depth_percent = 50; // Default 50%

// Status Reporting
uint8_t last_reported_hour = 255; // Invalid hour to force first report
char uart_buffer[256];

// UART Communication
volatile uint8_t uart_rx_byte;
volatile uint8_t uart_rx_flag = 0;

// Constants
#define SCALE_FACTOR 300                    // 300x faster than real time
#define SCALED_HOUR_MS (3600000/SCALE_FACTOR) // 12 seconds per scaled hour
#define PULSES_PER_REVOLUTION 20.0f
#define RESERVOIR_EMPTY_THRESHOLD 0
#define RESERVOIR_FULL_THRESHOLD 90

// PWM Options (duty cycle percentages)
const uint8_t PWM_OPTIONS[] = {0, 25, 50, 75, 100}; // Index 0 unused, 1-3 are options
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void); // Prototype for USART6
static void MX_TIM3_Init(void);       // Prototype for TIM3
static void MX_ADC1_Init(void);       // Prototype for ADC1
static void MX_TIM2_Init(void);       // Prototype for TIM2
static void MX_TIM5_Init(void);       // Prototype for TIM5
/* USER CODE BEGIN PFP */
// Prototype for the DIGITS_Display function (for device_selection == 3)
void DIGITS_Display(uint8_t DIGIT_A, uint8_t DIGIT_B);
// Prototype for ADC channel selection (for device_selection == 5)
static void ADC_Select_CH(int CH);

// This is the function that printf will call to send each character via UART2
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
  // Using huart2 for printf output
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Function to select ADC Channel - Provided by user
void ADC_Select_CH(int CH)
{
	ADC_ChannelConfTypeDef sConfig = {0};
    // Note: This function reconfigures a single channel for each call.
    // Ensure that all ADC channels used by your application are properly
    // initialized in MX_ADC1_Init or reconfigured here if multiple channels
    // are to be used sequentially.

	switch(CH)
	{
	case 0:
		sConfig.Channel = ADC_CHANNEL_0;
		sConfig.Rank = 1;
		if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); } break;
	case 1:
		sConfig.Channel = ADC_CHANNEL_1;
		sConfig.Rank = 1;
		if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); } break;
	case 2:
		sConfig.Channel = ADC_CHANNEL_2;
		sConfig.Rank = 1;
		if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); } break;
	case 3:
		sConfig.Channel = ADC_CHANNEL_3;
		sConfig.Rank = 1;
		if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); } break;
	case 4:
		sConfig.Channel = ADC_CHANNEL_4;
		sConfig.Rank = 1;
		if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); } break;
	case 5:
		sConfig.Channel = ADC_CHANNEL_5;
		sConfig.Rank = 1;
		if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); } break;
	case 6:
		sConfig.Channel = ADC_CHANNEL_6;
		sConfig.Rank = 1;
		if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); } break;
	case 7:
		sConfig.Channel = ADC_CHANNEL_7;
		sConfig.Rank = 1;
		if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); } break;
	case 8:
		sConfig.Channel = ADC_CHANNEL_8;
		sConfig.Rank = 1;
		if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); } break;
	case 9: // Potentiometer is expected on Channel 9 based on user's code
		sConfig.Channel = ADC_CHANNEL_9;
		sConfig.Rank = 1;
		sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES; // Ensure sampling time is set here too
		if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); } break;
	case 10:
		sConfig.Channel = ADC_CHANNEL_10;
		sConfig.Rank = 1;
		if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); } break;
	case 11:
		sConfig.Channel = ADC_CHANNEL_11;
		sConfig.Rank = 1;
		if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); } break;
	case 12:
		sConfig.Channel = ADC_CHANNEL_12;
		sConfig.Rank = 1;
		if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); } break;
	case 13:
		sConfig.Channel = ADC_CHANNEL_13;
		sConfig.Rank = 1;
		if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); } break;
	case 14:
		sConfig.Channel = ADC_CHANNEL_14;
		sConfig.Rank = 1;
		if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); } break;
	case 15:
		sConfig.Channel = ADC_CHANNEL_15;
		sConfig.Rank = 1;
		if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); } break;
	}
}

// DIGITS_Display function, provided by the user (for digital display mode)
void DIGITS_Display(uint8_t DIGIT_A, uint8_t DIGIT_B)
{
	 uint8_t DIGITA_VAL = 0x0F & DIGIT_A; //mask off higher4 bits
	 int Abit0 = (DIGITA_VAL ) & 1;  	// extract Abit0 of the 4-bit value
	 int Abit1 = (DIGITA_VAL >> 1) & 1;  // extract Abit1 of the 4-bit value
	 int Abit2 = (DIGITA_VAL >> 2) & 1;  // extract Abit2 of the 4-bit value
	 int Abit3 = (DIGITA_VAL >> 3) & 1;  // extract Abit3 of the 4-bit value

	 uint8_t DIGITB_VAL = 0x0F & DIGIT_B; //mask off higher4 bits
	 int Bbit0 = (DIGITB_VAL ) & 1;  	// extract Bbit0 of the 4-bit value
	 int Bbit1 = (DIGITB_VAL >> 1) & 1;  // extract Bbit1 of the 4-bit value
	 int Bbit2 = (DIGITB_VAL >> 2) & 1;  // extract Bbit2 of the 4-bit value
	 int Bbit3 = (DIGITB_VAL >> 3) & 1;  // extract Bbit3 of the 4-bit value

    // Use ifdef guards for pins that might not be universally defined
    #ifdef DIGIT_A0_Pin
	 if (Abit0 == (0)) { HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin, GPIO_PIN_RESET); } else { HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin, GPIO_PIN_SET); }
    #endif
    #ifdef DIGIT_A1_Pin
	 if (Abit1 == (0)) { HAL_GPIO_WritePin(GPIOB, DIGIT_A1_Pin, GPIO_PIN_RESET); } else { HAL_GPIO_WritePin(GPIOB, DIGIT_A1_Pin, GPIO_PIN_SET); }
    #endif
    #ifdef DIGIT_A2_Pin
	 if (Abit2 == (0)) { HAL_GPIO_WritePin(GPIOB, DIGIT_A2_Pin, GPIO_PIN_RESET); } else { HAL_GPIO_WritePin(GPIOB, DIGIT_A2_Pin, GPIO_PIN_SET); }
    #endif
    #ifdef DIGIT_A3_Pin
	 if (Abit3 == (0)) { HAL_GPIO_WritePin(GPIOB, DIGIT_A3_Pin, GPIO_PIN_RESET); } else { HAL_GPIO_WritePin(GPIOB, DIGIT_A3_Pin, GPIO_PIN_SET); }
    #endif

    #ifdef DIGIT_B0_Pin
	 if (Bbit0 == (0)) { HAL_GPIO_WritePin(GPIOC, DIGIT_B0_Pin, GPIO_PIN_RESET); } else { HAL_GPIO_WritePin(GPIOC, DIGIT_B0_Pin, GPIO_PIN_SET); }
    #endif
    #ifdef DIGIT_B1_Pin
	 if (Bbit1 == (0)) { HAL_GPIO_WritePin(GPIOC, DIGIT_B1_Pin, GPIO_PIN_RESET); } else { HAL_GPIO_WritePin(GPIOC, DIGIT_B1_Pin, GPIO_PIN_SET); }
    #endif
    #ifdef DIGIT_B2_Pin
	 if (Bbit2 == (0)) { HAL_GPIO_WritePin(GPIOC, DIGIT_B2_Pin, GPIO_PIN_RESET); } else { HAL_GPIO_WritePin(GPIOC, DIGIT_B2_Pin, GPIO_PIN_SET); }
    #endif
    #ifdef DIGIT_B3_Pin
	 if (Bbit3 == (0)) { HAL_GPIO_WritePin(GPIOC, DIGIT_B3_Pin, GPIO_PIN_RESET); } else { HAL_GPIO_WritePin(GPIOC, DIGIT_B3_Pin, GPIO_PIN_SET); }
    #endif
}


// --- Vineyard Irrigation Controller Function Prototypes ---
void Setup_Mode(void);
void Run_Mode(void);
void Configure_System(void);
void Load_Test_Case_1(void);
void Load_Test_Case_2(void);
void Load_Test_Case_3(void);
void Manual_Configuration(void);
void Display_Configuration_Summary(void);
void Set_Pipeline(Pipeline_t pipeline);
void Set_Motor_Speed(uint8_t percent);
void Set_Motor_Direction(uint8_t direction);
void Update_Water_Depth(void);
void Update_RPM(void);
void Update_BCD_Display(void);
void Set_RGB_LED(Pipeline_t pipeline);
void Send_Status_Report(void);
void Handle_Empty_Reservoir(void);
void Handle_End_Of_Cycle(void);
uint8_t Get_UART_Input(void);
uint8_t Read_Potentiometer(void);
uint32_t Read_Ultrasonic_Distance(void);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	// Remove unused variable
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
  MX_USART6_UART_Init(); // Initialize UART6 (used for receive test, and can be used for other TX if needed)
  MX_TIM2_Init();        // Initialize TIM2 (will be reconfigured per mode if needed)
  MX_TIM3_Init();        // Initialize TIM3 for DC motor
  MX_ADC1_Init();        // Initialize ADC1 for potentiometer
  MX_TIM5_Init();        // Initialize TIM5 for wall clock timer
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit(&huart6, (uint8_t*)"DEBUG: after init, before menu loop\r\n", 37, 100);
  // Initial global peripheral setup
  // TIM2 is initialized but not started here globally to avoid unintended PWM output.
  // It will be fully configured and started within each specific device_selection block.
  HAL_TIM_Base_Start(&htim2); // Start the timer base, but PWM output is not yet enabled globally.


  // Start TIM5 in interrupt mode for the wall clock functionality.
  __HAL_TIM_SET_COUNTER(&htim5, 0); // Explicitly set timer counter to 0
  HAL_TIM_Base_Start_IT(&htim5);

  // Initialize scaled clock variables for wall clock, done once at startup
  scaled_hours = 0;
  scaled_mins = 0;
  scaled_secs = 0;
  wall_clock_update_flag = 0; // Ensure flag is 0 at start

  // === VINEYARD IRRIGATION CONTROLLER INITIALIZATION ===
  
  // Start UART6 in interrupt receive mode for configuration
  HAL_UART_Receive_IT(&huart6, &byte, 1);
  
  // Initialize timers for motor control and servo
  HAL_TIM_Base_Init(&htim2);  // Servo motor
  HAL_TIM_Base_Init(&htim3);  // DC motor
  
  // Initialize servo to neutral position
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 500); // Start at inlet position
  
  // Initialize motor control pins
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  // Initialize motor to stopped state
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  
  // Initialize BCD display to show initial water level
  DIGITS_Display(5, 0); // Show 50% initial water level
  
  // Initialize RGB LED to off
  HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GRN_GPIO_Port, GRN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BLU_GPIO_Port, BLU_Pin, GPIO_PIN_RESET);
  
  // Initialize configuration to default values
  memset(&config, 0, sizeof(config));
  config_complete = 0;
  system_mode = MODE_SETUP;
  /* USER CODE END 2 */

  // === VINEYARD IRRIGATION CONTROLLER MAIN LOOP ===
  sprintf(uart_buffer, "=== VINEYARD IRRIGATION CONTROLLER ===\r\n");
  HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (system_mode == MODE_SETUP) {
      Setup_Mode();
    } else if (system_mode == MODE_RUN) {
      Run_Mode();
    }
    
    // Small delay to prevent tight loop
    HAL_Delay(10);
  }
  /* USER CODE END WHILE */
  /* USER CODE BEGIN 3 */
  // (Removed stray closing brace)
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2; // Using DIV2 as it appeared in one of your prior snippets
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B; // Keeping 8-bit resolution as per your potentiometer snippet
  hadc1.Init.ScanConvMode = ENABLE; // Keep ENABLE as it was in your potentiometer snippet's ADC init
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1; // Only one channel configured here as default
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9; // Default channel for initial setup, ADC_Select_CH will change this dynamically
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function (for Servo)
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
  // Default values for TIM2 (e.g., set for servo operation)
  htim2.Init.Prescaler = 16-1;
  htim2.Init.Period = 20000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = 500-1; // Initial pulse (e.g., for servo)
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function (for DC Motor)
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
  htim3.Init.Prescaler = 16-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000-1; // Period for DC motor PWM
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = 1200-1; // Initial pulse for DC motor PWM
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function (for Wall Clock)
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  // For a 200ms interrupt with a 16MHz HSI clock (scaled wall clock timing):
  // Prescaler = (16,000,000 / 10,000) - 1 = 1600 - 1 = 1599. (Creates a 10kHz timer clock)
  // Period = (2,000 / 1) - 1 = 1999. (Overflows every 200ms)
  htim5.Init.Prescaler = (16000000 / 10000) - 1; // Gives 10kHz counter clock
  htim5.Init.Period = (2000 - 1);                // Counts up to 2000 ticks, resulting in 200ms overflow
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART2 Initialization Function (for printf output)
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
  * @brief USART6 Initialization Function (for general UART test and potentiometer output)
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
  huart6.Init.Mode = UART_MODE_TX_RX; // Set to TX_RX to allow for HAL_UART_Receive_IT
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

  /*Configure GPIO pin Output Level for LD2_Pin (if defined) */
  #ifdef LD2_Pin
  // Configure LD2_Pin for Alternate Function (PWM output from TIM2)
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // Ensure LED is off initially
  #endif

  /*Configure GPIO pin Output Level for RGB_LEDs (if defined in main.h) */
  #ifdef BLU_Pin
  HAL_GPIO_WritePin(BLU_GPIO_Port, BLU_Pin, GPIO_PIN_RESET);
  #endif
  #ifdef GRN_Pin
  HAL_GPIO_WritePin(GRN_GPIO_Port, GRN_Pin, GPIO_PIN_RESET);
  #endif
  #ifdef RED_Pin
  HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);
  #endif

  /*Configure GPIO pin Output Level for Digital Display B pins (if defined in main.h) */
  #ifdef DIGIT_B0_Pin
  HAL_GPIO_WritePin(GPIOC, DIGIT_B0_Pin, GPIO_PIN_RESET);
  #endif
  #ifdef DIGIT_B1_Pin
  HAL_GPIO_WritePin(GPIOC, DIGIT_B1_Pin, GPIO_PIN_RESET);
  #endif
  #ifdef DIGIT_B2_Pin
  HAL_GPIO_WritePin(GPIOC, DIGIT_B2_Pin, GPIO_PIN_RESET);
  #endif

  /*Configure GPIO pin Output Level for Digital Display A pins and DIGIT_B3_Pin (if on GPIOB and defined in main.h) */
  #ifdef DIGIT_A0_Pin
  HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin, GPIO_PIN_RESET);
  #endif
  #ifdef DIGIT_A1_Pin
  HAL_GPIO_WritePin(GPIOB, DIGIT_A1_Pin, GPIO_PIN_RESET);
  #endif
  #ifdef DIGIT_A2_Pin
  HAL_GPIO_WritePin(GPIOB, DIGIT_A2_Pin, GPIO_PIN_RESET);
  #endif
  #ifdef DIGIT_A3_Pin
  HAL_GPIO_WritePin(GPIOB, DIGIT_A3_Pin, GPIO_PIN_RESET);
  #endif
  #ifdef DIGIT_B3_Pin // This was on GPIOB in your previous snippet
  HAL_GPIO_WritePin(GPIOB, DIGIT_B3_Pin, GPIO_PIN_RESET);
  #endif


  /*Configure GPIO pin : B1_Pin (User button often on Nucleo) */
  #ifdef B1_Pin
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
  #endif

  /*Configure GPIO pins : LD2_Pin (for LEDs, to be used with TIM2_CH1 for PWM) */
  #ifdef LD2_Pin
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // Set to Alternate Function Push-Pull
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2; // Specify TIM2 Alternate Function (e.g., PA5 is TIM2_CH1)
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
  #endif

  /*Configure GPIO pins for RGB_LEDs (if defined in main.h) */
  #ifdef BLU_Pin
  GPIO_InitStruct.Pin = BLU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLU_GPIO_Port, &GPIO_InitStruct);
  #endif
  #ifdef GRN_Pin
  GPIO_InitStruct.Pin = GRN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GRN_GPIO_Port, &GPIO_InitStruct);
  #endif
  #ifdef RED_Pin
  GPIO_InitStruct.Pin = RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RED_GPIO_Port, &GPIO_InitStruct);
  #endif

  /*Configure GPIO pins for Digital Display B pins (if defined in main.h) */
  #ifdef DIGIT_B0_Pin
  GPIO_InitStruct.Pin = DIGIT_B0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  #endif
  #ifdef DIGIT_B1_Pin
  GPIO_InitStruct.Pin = DIGIT_B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  #endif
  #ifdef DIGIT_B2_Pin
  GPIO_InitStruct.Pin = DIGIT_B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  #endif

  /*Configure GPIO pins for Digital Display A pins and DIGIT_B3_Pin (if defined in main.h) */
  #ifdef DIGIT_A0_Pin
  GPIO_InitStruct.Pin = DIGIT_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  #endif
  #ifdef DIGIT_A1_Pin
  GPIO_InitStruct.Pin = DIGIT_A1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  #endif
  #ifdef DIGIT_A2_Pin
  GPIO_InitStruct.Pin = DIGIT_A2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  #endif
  #ifdef DIGIT_A3_Pin
  GPIO_InitStruct.Pin = DIGIT_A3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  #endif
  #ifdef DIGIT_B3_Pin // This was on GPIOB in your previous snippet
  GPIO_InitStruct.Pin = DIGIT_B3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  #endif

  /*Configure GPIO pin : RPM_TICK_Pin (PB2) as EXTI input for RPM sensor */
  GPIO_InitStruct.Pin = RPM_TICK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RPM_TICK_GPIO_Port, &GPIO_InitStruct);
  HAL_NVIC_SetPriority(RPM_TICK_EXTI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(RPM_TICK_EXTI_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// TIM Callback function - Scaled Wall Clock (300x faster)
// TIM Callback function - Scaled Wall Clock (300x faster)
// TIM Callback function - Scaled Wall Clock (300x faster)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if((htim->Instance == TIM5))
    {
        // Increment scaled time (every second in real time = 5 minutes scaled time)
        // TIM5 configured to interrupt every 200ms, so 5 interrupts = 1 second real time
        static uint8_t interrupt_count = 0;
        interrupt_count++;

        if (interrupt_count >= 5) { // 1 second real time = 5 minutes scaled time
            interrupt_count = 0;
            scaled_mins += 5;

            if (scaled_mins >= 60) {
                scaled_hours += 1;
                scaled_mins = 0;
                wall_clock_update_flag = 1; // Signal hour change

                // --- MODIFIED LOGIC FOR END OF DAY ---
                // This condition triggers exactly when the clock becomes 24:00
                if (scaled_hours == 25) {
                    // Perform end-of-cycle actions
                    Handle_End_Of_Cycle();

                    // Reset scaled hours to 0 after the cycle ends
                    // This ensures the clock effectively "rolls over" to a new day after handling the end.
                    scaled_hours = 0;
                    scaled_mins = 0; // Also reset minutes
                    // The wall_clock_update_flag is already set above, no need to set again here
                    // unless you specifically want another update signal for the 00:00 state.

                    return; // Exit the callback after handling the end of cycle to prevent
                            // further execution of time-related logic for this interrupt
                            // if Handle_End_Of_Cycle takes time or alters system state significantly.
                }
                // --- END MODIFIED LOGIC ---
            }
        }
    }
}

// UART Receive Complete Callback - handles received data for USART6
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART6)
	{
		// Store received byte and set flag
		uart_rx_byte = byte;
		uart_rx_flag = 1;
		
		// Restart the interrupt receive for the next byte
		HAL_UART_Receive_IT(&huart6, &byte, 1);
	}
}

// --- EXTI CALLBACK FOR RPM SENSOR ---
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == RPM_TICK_Pin){
        rpm_counter += 1;
        rpm_tick_flag = 1;
    }
}

// === VINEYARD IRRIGATION CONTROLLER IMPLEMENTATION ===

// SETUP Mode - Configuration Interface
void Setup_Mode(void)
{
    static uint8_t setup_initialized = 0;
    
    if (!setup_initialized) {
        // Initialize SETUP mode
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // Green LED OFF
        sprintf(uart_buffer, "\r\n=== SETUP MODE ===\r\n");
        HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
        
        // Stop motor
        Set_Motor_Speed(0);
        
        // Start configuration
        Configure_System();
        setup_initialized = 1;
    }
    
    // Wait for B1 button press to start RUN mode
    if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET && config_complete) {
        // Button pressed and config complete
        HAL_Delay(50); // Debounce
        if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) {
            system_mode = MODE_RUN;
            setup_initialized = 0;
            scaled_hours = 0;
            scaled_mins = 0;
            last_reported_hour = 255;
            sprintf(uart_buffer, "\r\n=== STARTING RUN MODE ===\r\n");
            HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
        }
    }
    
    // Flash green LED when setup complete
    if (config_complete) {
        static uint32_t last_flash = 0;
        if (HAL_GetTick() - last_flash > 500) {
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            last_flash = HAL_GetTick();
        }
    }
}

// Configuration System - Test Cases + Manual Setup
void Configure_System(void)
{
    sprintf(uart_buffer, "\r\n=== CONFIGURATION OPTIONS ===\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    sprintf(uart_buffer, "1) Test Case 1 - Basic Sequential Operation\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    sprintf(uart_buffer, "2) Test Case 2 - Overlapping Operation\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    sprintf(uart_buffer, "3) Test Case 3 - Potentiometer Manual PWM Test\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    sprintf(uart_buffer, "4) Manual Configuration\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    sprintf(uart_buffer, "Select option (1-4): ");
    HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    
    uint8_t choice = Get_UART_Input();
    
    switch (choice) {
        case 1:
            Load_Test_Case_1();
            break;
        case 2:
            Load_Test_Case_2();
            break;
        case 3:
            Load_Test_Case_3();
            break;
        case 4:
            Manual_Configuration();
            break;
        default:
            sprintf(uart_buffer, "Invalid choice, using Test Case 1...\r\n");
            HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
            Load_Test_Case_1();
            break;
    }
    
    // Display configuration summary
    Display_Configuration_Summary();
    
    sprintf(uart_buffer, "\r\nConfiguration complete! Press BLUE button to start.\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    
    config_complete = 1;
}

// Test Case 1 - Basic Sequential Operation
void Load_Test_Case_1(void)
{
    sprintf(uart_buffer, "\r\n=== LOADING TEST CASE 1 - BASIC SEQUENTIAL ===\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    
    // Inlet: Manual control (potentiometer), 0-2 hours (early morning fill)
    config.inlet_pwm_option = 0;  // Manual control
    config.inlet_start_hour = 0;
    config.inlet_stop_hour = 2;
    
    // Zone 1: 50% PWM, 3-5 hours (morning irrigation)
    config.zone_order[0] = 1;
    config.zone_pwm_option[0] = 2;  // 50% PWM
    config.zone_start_hour[0] = 3;
    config.zone_stop_hour[0] = 5;
    
    // Zone 2: 25% PWM, 6-8 hours (gentle irrigation)
    config.zone_order[1] = 2;
    config.zone_pwm_option[1] = 1;  // 25% PWM
    config.zone_start_hour[1] = 6;
    config.zone_stop_hour[1] = 8;
    
    // Zone 3: 75% PWM, 9-11 hours (high pressure irrigation)
    config.zone_order[2] = 3;
    config.zone_pwm_option[2] = 3;  // 75% PWM
    config.zone_start_hour[2] = 9;
    config.zone_stop_hour[2] = 11;
    
    sprintf(uart_buffer, "Test Case 1 loaded: Sequential operation with manual inlet control\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
}

// Test Case 2 - Overlapping Operation
void Load_Test_Case_2(void)
{
    sprintf(uart_buffer, "\r\n=== LOADING TEST CASE 2 - OVERLAPPING OPERATION ===\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    
    // Inlet: 25% PWM, 22-1 hours (overnight fill, wraps around midnight)
    config.inlet_pwm_option = 1;  // 25% PWM
    config.inlet_start_hour = 22;
    config.inlet_stop_hour = 1;
    
    // Zone 1: 75% PWM, 8-10 hours (morning high pressure)
    config.zone_order[0] = 1;
    config.zone_pwm_option[0] = 3;  // 75% PWM
    config.zone_start_hour[0] = 8;
    config.zone_stop_hour[0] = 10;
    
    // Zone 2: 50% PWM, 10-12 hours (overlaps with Zone 1 end)
    config.zone_order[1] = 2;
    config.zone_pwm_option[1] = 2;  // 50% PWM
    config.zone_start_hour[1] = 10;
    config.zone_stop_hour[1] = 12;
    
    // Zone 3: 25% PWM, 14-16 hours (afternoon gentle irrigation)
    config.zone_order[2] = 3;
    config.zone_pwm_option[2] = 1;  // 25% PWM
    config.zone_start_hour[2] = 14;
    config.zone_stop_hour[2] = 16;
    
    sprintf(uart_buffer, "Test Case 2 loaded: Overlapping operation with overnight inlet\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
}

// Test Case 3 - Potentiometer Manual PWM Test
void Load_Test_Case_3(void)
{
    sprintf(uart_buffer, "\r\n=== LOADING TEST CASE 3 - POTENTIOMETER MANUAL PWM TEST ===\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    
    // Inlet: Manual control (potentiometer), 0-6 hours (extended morning fill for testing)
    config.inlet_pwm_option = 0;  // Manual control
    config.inlet_start_hour = 0;
    config.inlet_stop_hour = 6;
    
    // Zone 1: Manual control (potentiometer), 8-10 hours (manual zone control)
    config.zone_order[0] = 1;
    config.zone_pwm_option[0] = 0;  // Manual control (using potentiometer)
    config.zone_start_hour[0] = 8;
    config.zone_stop_hour[0] = 10;
    
    // Zone 2: Manual control (potentiometer), 12-14 hours (afternoon manual control)
    config.zone_order[1] = 2;
    config.zone_pwm_option[1] = 0;  // Manual control (using potentiometer)
    config.zone_start_hour[1] = 12;
    config.zone_stop_hour[1] = 14;
    
    // Zone 3: Manual control (potentiometer), 16-18 hours (evening manual control)
    config.zone_order[2] = 3;
    config.zone_pwm_option[2] = 0;  // Manual control (using potentiometer)
    config.zone_start_hour[2] = 16;
    config.zone_stop_hour[2] = 18;
    
    sprintf(uart_buffer, "Test Case 3 loaded: All operations use potentiometer for manual PWM control\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    sprintf(uart_buffer, "*** Turn potentiometer to adjust PWM during operation ***\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
}

// Manual Configuration - Interactive UART Setup
void Manual_Configuration(void)
{
    sprintf(uart_buffer, "\r\n=== MANUAL CONFIGURATION ===\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    
    // Part A - Pipeline & Pump Configuration
    sprintf(uart_buffer, "\r\n=== PART A: PIPELINE & PUMP CONFIGURATION ===\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    
    // Get inlet PWM option
    sprintf(uart_buffer, "Inlet Pump Motor PWM Option (0=Manual, 1=25%%, 2=50%%, 3=75%%): ");
    HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    config.inlet_pwm_option = Get_UART_Input();
    
    // Get zone order and PWM options
    for (int i = 0; i < 3; i++) {
        sprintf(uart_buffer, "Zone %d Choice (1-3): ", i + 1);
        HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
        config.zone_order[i] = Get_UART_Input();
        
        sprintf(uart_buffer, "Zone %d PWM Option (0=Manual, 1=25%%, 2=50%%, 3=75%%): ", i + 1);
        HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
        config.zone_pwm_option[i] = Get_UART_Input();
    }
    
    // Part B - Schedule Configuration
    sprintf(uart_buffer, "\r\n=== PART B: SCHEDULE CONFIGURATION ===\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    
    // Get inlet schedule
    sprintf(uart_buffer, "Inlet START Time (HH 0-23): ");
    HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    config.inlet_start_hour = Get_UART_Input();
    
    sprintf(uart_buffer, "Inlet STOP Time (HH 0-23): ");
    HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    config.inlet_stop_hour = Get_UART_Input();
    
    // Get zone schedules
    for (int i = 0; i < 3; i++) {
        sprintf(uart_buffer, "Zone %d START Time (HH 0-23): ", i + 1);
        HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
        config.zone_start_hour[i] = Get_UART_Input();
        
        sprintf(uart_buffer, "Zone %d STOP Time (HH 0-23): ", i + 1);
        HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
        config.zone_stop_hour[i] = Get_UART_Input();
    }
    
    sprintf(uart_buffer, "Manual configuration complete!\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
}

// Display Configuration Summary
void Display_Configuration_Summary(void)
{
    sprintf(uart_buffer, "\r\n=== CONFIGURATION SUMMARY ===\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    
    // Show inlet configuration
    const char* pwm_labels[] = {"Manual", "25%", "50%", "75%"};
    sprintf(uart_buffer, "Inlet: %s PWM, Schedule: %02d:00-%02d:00\r\n", 
            pwm_labels[config.inlet_pwm_option], 
            config.inlet_start_hour, config.inlet_stop_hour);
    HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    
    // Show zone configurations
    for (int i = 0; i < 3; i++) {
        const char* zone_pwm_str = (config.zone_pwm_option[i] == 0) ? "Manual" : pwm_labels[config.zone_pwm_option[i]];
        sprintf(uart_buffer, "Zone %d: Target=Zone%d, %s PWM, Schedule=%02d:00-%02d:00\r\n", 
                i + 1, config.zone_order[i], 
                zone_pwm_str, 
                config.zone_start_hour[i], config.zone_stop_hour[i]);
        HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    }
    
    // Show time scaling info
    sprintf(uart_buffer, "\r\n*** TIME SCALING: 300x faster (24h = 4.8 min real time) ***\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
}

// RUN Mode - Main Irrigation Operation
void Run_Mode(void)
{
    static uint8_t run_initialized = 0;
    
    if (!run_initialized) {
        // Initialize RUN mode
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); // Green LED ON
        sprintf(uart_buffer, "RUN MODE: Scaled Wall Clock 00:00\r\n");
        HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
        run_initialized = 1;
    }
    
    // Check for end of 24-hour cycle
    if (scaled_hours >= 24) {
        Handle_End_Of_Cycle();
        return;
    }
    
    // Update sensor readings
    Update_Water_Depth();
    Update_RPM();
    Update_BCD_Display();
    
    // Check for hour change to send status report
    if (wall_clock_update_flag) {
        wall_clock_update_flag = 0;
        Send_Status_Report();
    }
    
    // Check if we should be operating at current time
    uint8_t should_operate = 0;
    Pipeline_t target_pipeline = PIPELINE_INLET;
    uint8_t target_pwm = 0;
    
    // Check inlet schedule (time-based, reservoir level dependent)
    uint8_t inlet_time_active = 0;
    if (config.inlet_start_hour <= config.inlet_stop_hour) {
        // Normal case: start < stop (e.g., 0-2 hours)
        inlet_time_active = (scaled_hours >= config.inlet_start_hour && scaled_hours < config.inlet_stop_hour);
    } else {
        // Wrap-around case: start > stop (e.g., 22-1 hours)
        inlet_time_active = (scaled_hours >= config.inlet_start_hour || scaled_hours < config.inlet_stop_hour);
    }
    
    if (inlet_time_active) {
        // Check if reservoir is less than 90% full
        if (water_depth_percent < RESERVOIR_FULL_THRESHOLD) {
            should_operate = 1;
            target_pipeline = PIPELINE_INLET;
            if (config.inlet_pwm_option == 0) {
                target_pwm = Read_Potentiometer(); // Manual control
            } else {
                target_pwm = PWM_OPTIONS[config.inlet_pwm_option];
            }
        }
    }
    
    // Check zone schedules (zones override inlet)
    for (int i = 0; i < 3; i++) {
        uint8_t zone_time_active = 0;
        if (config.zone_start_hour[i] <= config.zone_stop_hour[i]) {
            // Normal case: start < stop
            zone_time_active = (scaled_hours >= config.zone_start_hour[i] && scaled_hours < config.zone_stop_hour[i]);
        } else {
            // Wrap-around case: start > stop
            zone_time_active = (scaled_hours >= config.zone_start_hour[i] || scaled_hours < config.zone_stop_hour[i]);
        }
        
        if (zone_time_active) {
            // Check if reservoir has water
            if (water_depth_percent > RESERVOIR_EMPTY_THRESHOLD) {
                should_operate = 1;
                target_pipeline = (Pipeline_t)config.zone_order[i];
                if (config.zone_pwm_option[i] == 0) {
                    target_pwm = Read_Potentiometer(); // Manual control
                } else {
                    target_pwm = PWM_OPTIONS[config.zone_pwm_option[i]];
                }
                break; // First matching zone takes priority
            }
        }
    }
    
    // Apply operation
    if (should_operate) {
        Set_Pipeline(target_pipeline);
        Set_RGB_LED(target_pipeline);
        Set_Motor_Direction(1); // Same direction for both inlet and zones
        Set_Motor_Speed(target_pwm);
        motor_running = 1;
        motor_pwm_percent = target_pwm;
        current_pipeline = target_pipeline;
        
        // Debug output for motor operation
        const char* pipeline_names[] = {"Inlet", "Zone1", "Zone2", "Zone3"};
        // sprintf(uart_buffer, "MOTOR ON: %s, %d%%PWM, Dir=%d, Water=%d%%\r\n", 
        //         pipeline_names[target_pipeline], target_pwm, 
        //         1, water_depth_percent);
        // HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    } else {
        // Only turn off if currently running (avoid spam)
        if (motor_running) {
            Set_Motor_Speed(0);
            motor_running = 0;
            motor_pwm_percent = 0;
            // sprintf(uart_buffer, "MOTOR OFF: No operation scheduled at %02d:00\r\n", scaled_hours);
            // HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
        }
    }
    
    // Check for empty reservoir during irrigation
    if (motor_running && target_pipeline != PIPELINE_INLET && water_depth_percent <= RESERVOIR_EMPTY_THRESHOLD) {
        Handle_Empty_Reservoir();
    }
}

// Set Pipeline - Control servo to select pipeline
void Set_Pipeline(Pipeline_t pipeline)
{
    // Servo positions: 500-2500 microseconds pulse width
    // Inlet=500, Zone1=1000, Zone2=1500, Zone3=2000
    uint16_t servo_positions[] = {500, 1000, 1500, 2000};
    
    if (pipeline <= PIPELINE_ZONE3) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, servo_positions[pipeline]);
        HAL_Delay(500); // Allow servo to move
    }
}

// Set Motor Speed - Control DC motor PWM
void Set_Motor_Speed(uint8_t percent)
{
    if (percent > 100) percent = 100;
    
    // Convert percentage to TIM3 PWM value
    uint16_t pwm_value = (uint16_t)((percent * (2000 - 1)) / 100);
    
    if (percent == 0) {
        // Stop motor by setting both direction pins low and stopping PWM
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
    } else {
        // Set PWM for speed control and apply direction
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_value);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm_value);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
        
        // Set direction using GPIO pins
        if (motor_direction == 0) {
            // Forward (zones): PA6=HIGH, PB0=LOW
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        } else {
            // Reverse (inlet): PA6=LOW, PB0=HIGH
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        }
    }
}

// Set Motor Direction - Control motor direction
void Set_Motor_Direction(uint8_t direction)
{
    motor_direction = direction;
    // Direction control is now handled in Set_Motor_Speed function
    // 0 = Forward (zones), 1 = Reverse (inlet)
}

// Update Water Depth - Read ultrasonic sensor
void Update_Water_Depth(void)
{
    // Simulate ultrasonic sensor reading
    // In real implementation, this would trigger ultrasonic sensor
    uint32_t distance = Read_Ultrasonic_Distance();
    
    // Convert distance to water depth percentage
    // Assuming: 0cm = 100% full, 30cm = 0% empty
    if (distance <= 0) {
        water_depth_percent = 100;
    } else if (distance >= 30) {
        water_depth_percent = 0;
    } else {
        water_depth_percent = (uint8_t)(100 - (distance * 100 / 30));
    }
    
    // Simulate water consumption during irrigation
    if (motor_running && current_pipeline != PIPELINE_INLET && water_depth_percent > 0) {
        static uint32_t last_consumption = 0;
        if (HAL_GetTick() - last_consumption > 5000) { // Every 5 seconds
            if (water_depth_percent > 0) {
                water_depth_percent -= 1; // Decrease water level
            }
            last_consumption = HAL_GetTick();
        }
    }
    
    // Simulate water filling during inlet operation
    if (motor_running && current_pipeline == PIPELINE_INLET && water_depth_percent < 100) {
        static uint32_t last_fill = 0;
        if (HAL_GetTick() - last_fill > 3000) { // Every 3 seconds
            if (water_depth_percent < 100) {
                water_depth_percent += 2; // Increase water level
            }
            last_fill = HAL_GetTick();
        }
    }
}

// Update RPM - Calculate motor RPM from pulse count
void Update_RPM(void)
{
    static uint32_t last_rpm_calc = 0;
    static uint16_t last_pulse_count = 0;
    
    if (HAL_GetTick() - last_rpm_calc >= 1000) { // Calculate every second
        uint16_t pulse_diff = rpm_counter - last_pulse_count;
        current_rpm = (uint16_t)((pulse_diff * 60.0f) / PULSES_PER_REVOLUTION);
        
        last_pulse_count = rpm_counter;
        last_rpm_calc = HAL_GetTick();
    }
}

// Update BCD Display - Show water depth percentage
void Update_BCD_Display(void)
{
    uint8_t tens = water_depth_percent / 10;
    uint8_t ones = water_depth_percent % 10;
    DIGITS_Display(tens, ones);
}

// Set RGB LED - Indicate active pipeline
void Set_RGB_LED(Pipeline_t pipeline)
{
    // Turn off all LEDs first
    HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GRN_GPIO_Port, GRN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BLU_GPIO_Port, BLU_Pin, GPIO_PIN_RESET);
    
    switch (pipeline) {
        case PIPELINE_INLET:
            // Purple (Red + Blue)
            HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(BLU_GPIO_Port, BLU_Pin, GPIO_PIN_SET);
            break;
        case PIPELINE_ZONE1:
            // Red
            HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_SET);
            break;
        case PIPELINE_ZONE2:
            // Green
            HAL_GPIO_WritePin(GRN_GPIO_Port, GRN_Pin, GPIO_PIN_SET);
            break;
        case PIPELINE_ZONE3:
            // Blue
            HAL_GPIO_WritePin(BLU_GPIO_Port, BLU_Pin, GPIO_PIN_SET);
            break;
    }
}

// Send Status Report - Hourly status via UART
void Send_Status_Report(void)
{
    if (last_reported_hour == scaled_hours) {
        return; // Already reported this hour
    }
    
    const char* pipeline_names[] = {"Inlet", "Zone1", "Zone2", "Zone3"};
    const char* pipeline_str = "";
    
    if (motor_running) {
        pipeline_str = pipeline_names[current_pipeline];
    }
    
    sprintf(uart_buffer, "Hour %02d | %s | %d%%PWM | %dRPM | %d%% Water\r\n", 
            scaled_hours, 
            motor_running ? pipeline_str : "",
            motor_running ? motor_pwm_percent : 0,
            motor_running ? current_rpm : 0,
            water_depth_percent);
    
    HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    last_reported_hour = scaled_hours;
}

// Handle Empty Reservoir - Error condition
void Handle_Empty_Reservoir(void)
{
    // Turn off motor immediately
    Set_Motor_Speed(0);
    motor_running = 0;
    
    // Turn off green LED
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    
    // Display error message
    sprintf(uart_buffer, "\r\n*** RESERVOIR IS EMPTY ***\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    
    // Flash RGB LED white
    while (1) {
        HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GRN_GPIO_Port, GRN_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BLU_GPIO_Port, BLU_Pin, GPIO_PIN_SET);
        HAL_Delay(500);
        HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GRN_GPIO_Port, GRN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BLU_GPIO_Port, BLU_Pin, GPIO_PIN_RESET);
        HAL_Delay(500);
    }
}

// Handle End of Cycle - 24-hour completion
void Handle_End_Of_Cycle(void) {
    // 1. Stop all active motors
    Set_Motor_Speed(0); // Set DC motor speed to 0
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // Ensure motor direction pins are off
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

    // Stop servo movement (return to a safe default if needed, or simply stop PWM)
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

    // 2. Blink RGB LED white (Red + Green + Blue)
    // Blink for a few seconds
    for (int i = 0; i < 10; i++) { // Blink 5 times (on/off cycles)
        HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GRN_GPIO_Port, GRN_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BLU_GPIO_Port, BLU_Pin, GPIO_PIN_SET);
        HAL_Delay(200); // White for 200ms

        HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GRN_GPIO_Port, GRN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BLU_GPIO_Port, BLU_Pin, GPIO_PIN_RESET);
        HAL_Delay(200); // Off for 200ms
    }

    // Ensure LED is off at the very end
    HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GRN_GPIO_Port, GRN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BLU_GPIO_Port, BLU_Pin, GPIO_PIN_RESET);

    // 3. Stop the Wall Clock Timer (TIM5)
    HAL_TIM_Base_Stop_IT(&htim5);

    // 4. Optionally, send a final status report or message
    sprintf(uart_buffer, "System: Irrigation cycle complete. Entering idle mode.\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);

    // 5. Enter an infinite loop to halt the system or set a new system mode
    // If the system should truly "end" and not restart, an infinite loop is common.
    // If it should transition to a specific "ended" mode, set a flag and handle it in main loop.
    system_mode = MODE_SETUP; // For example, transition back to setup mode if desired to allow re-configuration.
                              // If it should truly stop, an infinite loop `while(1) {}` is used here.
    // If you want it to completely stop and do nothing else:
    // while(1) {
    //     // System is halted
    // }
}

// Get UART Input - Read numeric input from UART
uint8_t Get_UART_Input(void)
{
    char input_buffer[10] = {0};
    int index = 0;
    
    while (1) {
        // Wait for UART input
        while (!uart_rx_flag) {
            HAL_Delay(1);
        }
        
        uart_rx_flag = 0;
        char received = uart_rx_byte;
        
        // Echo the character
        HAL_UART_Transmit(&huart6, (uint8_t*)&received, 1, HAL_MAX_DELAY);
        
        if (received == '\r' || received == '\n') {
            // End of input
            HAL_UART_Transmit(&huart6, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
            input_buffer[index] = '\0';
            return (uint8_t)atoi(input_buffer);
        } else if (received >= '0' && received <= '9' && index < 9) {
            // Valid numeric input
            input_buffer[index++] = received;
        } else if (received == '\b' && index > 0) {
            // Backspace
            index--;
            HAL_UART_Transmit(&huart6, (uint8_t*)" \b", 2, HAL_MAX_DELAY);
        }
    }
}

// Read Potentiometer - Get manual PWM value
uint8_t Read_Potentiometer(void)
{
    uint16_t adc_value = 0;
    
    ADC_Select_CH(9);
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
        adc_value = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
    
    // Convert 8-bit ADC to percentage (0-100)
    return (uint8_t)((adc_value * 100) / 255);
}

// Read Ultrasonic Distance - Simulate sensor reading
uint32_t Read_Ultrasonic_Distance(void)
{
    // Simulate ultrasonic sensor reading
    // In real implementation, this would use timer input capture
    // For simulation, use a pseudo-random value based on motor operation
    static uint32_t simulated_distance = 15; // Start at 50% water level
    
    // Return the simulated distance (0-30 cm)
    return simulated_distance;
}

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
  * where the assert_param error has occurred.
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
