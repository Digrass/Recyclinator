/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "libjpeg.h"
#include "app_touchgfx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32h735g_discovery_ospi.h"
#include "stm32h7xx_hal_ospi.h"
#include "shared_sensor_types.h" //C인 main.c과 C++인 Model 사이에 enum을 공유하기 위한 브릿지 헤더

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PROXIMITY_POLLING_INTERVAL_MS 2000 // 폴링 주기
#define DETECTION_DISTANCE_THRESHOLD_CM 50 // 사람 감지 거리 임계값
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

CRC_HandleTypeDef hcrc;

DMA2D_HandleTypeDef hdma2d;

LTDC_HandleTypeDef hltdc;

OSPI_HandleTypeDef hospi1;
OSPI_HandleTypeDef hospi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim23;

/* Definitions for ProximityTask */
osThreadId_t ProximityTaskHandle;
const osThreadAttr_t ProximityTask_attributes = {
  .name = "ProximityTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for TouchGFXTask */
osThreadId_t TouchGFXTaskHandle;
const osThreadAttr_t TouchGFXTask_attributes = {
  .name = "TouchGFXTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for videoTask */
osThreadId_t videoTaskHandle;
const osThreadAttr_t videoTask_attributes = {
  .name = "videoTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for sensorTask */
osThreadId_t sensorTaskHandle;
const osThreadAttr_t sensorTask_attributes = {
  .name = "sensorTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* USER CODE BEGIN PV */
volatile uint8_t g_echo_state_us0 = 0; // 0: Idle/Rising edge 대기, 1: Falling edge 대기, 2: 측정 완료
volatile uint32_t g_ic_rising_tick_us0 = 0;
volatile uint32_t g_ic_falling_tick_us0 = 0;

volatile uint8_t g_echo_state_us1 = 0;
volatile uint32_t g_ic_rising_tick_us1 = 0;
volatile uint32_t g_ic_falling_tick_us1 = 0;

volatile uint8_t g_echo_state_us2 = 0;
volatile uint32_t g_ic_rising_tick_us2 = 0;
volatile uint32_t g_ic_falling_tick_us2 = 0;

TIM_HandleTypeDef* servo_htim[NUM_ACTIVE_SERVOS] = {
	TIM_SERVO_0,
	TIM_SERVO_1,
	/*
	TIM_SERVO_2,
	TIM_SERVO_3,
	TIM_SERVO_4
	*/
};

uint32_t servo_channel[NUM_ACTIVE_SERVOS] = {
	TIM_CHANNEL_SERVO_0,
	TIM_CHANNEL_SERVO_1,
	/*
	TIM_CHANNEL_SERVO_2,
	TIM_CHANNEL_SERVO_3,
	TIM_CHANNEL_SERVO_4
	*/
};

osMessageQueueId_t sensorRequestQueueHandle; // Model -> SensorTask
osMessageQueueId_t sensorResultQueueHandle;  // SensorTask -> Model

osMessageQueueId_t modelToProximityQueueHandle;
osMessageQueueId_t proximityToModelQueueHandle;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_DMA2D_Init(void);
static void MX_LTDC_Init(void);
static void MX_OCTOSPI1_Init(void);
static void MX_OCTOSPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM23_Init(void);
void StartProximityTask(void *argument);
extern void TouchGFX_Task(void *argument);
extern void videoTaskFunc(void *argument);
void StartSensorTask(void *argument);

/* USER CODE BEGIN PFP */
void DWT_Delay_Init(void);
void DWT_Delay_us(uint32_t microseconds);
void Start_All_PWM(void);
void set_servo0_angle(uint16_t angle_pulse_value);
void set_servo1_angle(uint16_t angle_pulse_value);
void Trigger_US0();
void Trigger_US1();
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void Start_US0_Measurement();
void Start_US1_Measurement();
float Get_US0_Distance_NonBlocking();
float Get_US1_Distance_NonBlocking();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_DMA2D_Init();
  MX_LTDC_Init();
  MX_OCTOSPI1_Init();
  MX_OCTOSPI2_Init();
  MX_LIBJPEG_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM23_Init();
  MX_TouchGFX_Init();
  /* Call PreOsInit function */
  MX_TouchGFX_PreOSInit();
  /* USER CODE BEGIN 2 */
  Start_All_PWM();
  DWT_Delay_Init();

  if (HAL_TIM_IC_Start_IT(&htim23, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_IC_Start_IT(&htim23, TIM_CHANNEL_3) != HAL_OK) {
	Error_Handler();
  }
  if (HAL_TIM_IC_Start_IT(&htim23, TIM_CHANNEL_1) != HAL_OK) {
	Error_Handler();
  }

  const osMessageQueueAttr_t sensorRequestQueue_attributes = {
    .name = "sensorRequestQueue"
  };
  const osMessageQueueAttr_t sensorResultQueue_attributes = {
    .name = "sensorResultQueue"
  };
  const osMessageQueueAttr_t modelToProximityQueue_attributes = {
    .name = "ModelToProxQ"
  };
  const osMessageQueueAttr_t proximityToModelQueue_attributes = {
    .name = "ProxToModelQ"
  };
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  sensorRequestQueueHandle = osMessageQueueNew(8, sizeof(CSensorRequestMessage_t), &sensorRequestQueue_attributes);
  if (sensorRequestQueueHandle == NULL) {
	  Error_Handler();
  }

  sensorResultQueueHandle = osMessageQueueNew(8, sizeof(CSensorResultMessage_t), &sensorResultQueue_attributes);
   if (sensorResultQueueHandle == NULL) {
     Error_Handler();
   }

   modelToProximityQueueHandle = osMessageQueueNew(4, sizeof(ModelToProximityMsg_t), &modelToProximityQueue_attributes);
   if (modelToProximityQueueHandle == NULL) {
     Error_Handler();
   }

   proximityToModelQueueHandle = osMessageQueueNew(4, sizeof(ProximityToModelMsg_t), &proximityToModelQueue_attributes);
   if (proximityToModelQueueHandle == NULL) {
     Error_Handler();
   }

  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ProximityTask */
  ProximityTaskHandle = osThreadNew(StartProximityTask, NULL, &ProximityTask_attributes);

  /* creation of TouchGFXTask */
  TouchGFXTaskHandle = osThreadNew(TouchGFX_Task, NULL, &TouchGFXTask_attributes);

  /* creation of videoTask */
  videoTaskHandle = osThreadNew(videoTaskFunc, NULL, &videoTask_attributes);

  /* creation of sensorTask */
  sensorTaskHandle = osThreadNew(StartSensorTask, NULL, &sensorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 44;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_OSPI;
  PeriphClkInitStruct.PLL2.PLL2M = 5;
  PeriphClkInitStruct.PLL2.PLL2N = 80;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.OspiClockSelection = RCC_OSPICLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_R2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB888;
  hdma2d.Init.OutputOffset = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 40;
  hltdc.Init.VerticalSync = 9;
  hltdc.Init.AccumulatedHBP = 53;
  hltdc.Init.AccumulatedVBP = 11;
  hltdc.Init.AccumulatedActiveW = 533;
  hltdc.Init.AccumulatedActiveH = 283;
  hltdc.Init.TotalWidth = 565;
  hltdc.Init.TotalHeigh = 285;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 480;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 272;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB888;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0x70000000;
  pLayerCfg.ImageWidth = 480;
  pLayerCfg.ImageHeight = 272;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */
  BSP_OSPI_NOR_Init_t ospi_nor_int;
  /* USER CODE END OCTOSPI1_Init 0 */

  OSPIM_CfgTypeDef sOspiManagerCfg = {0};

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 4;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MACRONIX;
  hospi1.Init.DeviceSize = 32;
  hospi1.Init.ChipSelectHighTime = 2;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
  hospi1.Init.ClockPrescaler = 2;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  hospi1.Init.ChipSelectBoundary = 0;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  hospi1.Init.MaxTran = 0;
  hospi1.Init.Refresh = 0;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  sOspiManagerCfg.ClkPort = 1;
  sOspiManagerCfg.DQSPort = 1;
  sOspiManagerCfg.NCSPort = 1;
  sOspiManagerCfg.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  sOspiManagerCfg.IOHighPort = HAL_OSPIM_IOPORT_1_HIGH;
  if (HAL_OSPIM_Config(&hospi1, &sOspiManagerCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */
  HAL_OSPI_DeInit(&hospi1);
  ospi_nor_int.InterfaceMode = BSP_OSPI_NOR_OPI_MODE;
  ospi_nor_int.TransferRate  = BSP_OSPI_NOR_DTR_TRANSFER;
  BSP_OSPI_NOR_DeInit(0);
  if(BSP_OSPI_NOR_Init(0, &ospi_nor_int) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
  if(BSP_OSPI_NOR_EnableMemoryMappedMode(0) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
  /* USER CODE END OCTOSPI1_Init 2 */

}

/**
  * @brief OCTOSPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI2_Init(void)
{

  /* USER CODE BEGIN OCTOSPI2_Init 0 */
  BSP_OSPI_RAM_Init_t ospi_ram_init;
  /* USER CODE END OCTOSPI2_Init 0 */

  OSPIM_CfgTypeDef sOspiManagerCfg = {0};
  OSPI_HyperbusCfgTypeDef sHyperBusCfg = {0};

  /* USER CODE BEGIN OCTOSPI2_Init 1 */
  OSPI_HyperbusCmdTypeDef sCommand = {0};
  OSPI_MemoryMappedTypeDef sMemMappedCfg = {0};
  /* USER CODE END OCTOSPI2_Init 1 */
  /* OCTOSPI2 parameter configuration*/
  hospi2.Instance = OCTOSPI2;
  hospi2.Init.FifoThreshold = 4;
  hospi2.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi2.Init.MemoryType = HAL_OSPI_MEMTYPE_HYPERBUS;
  hospi2.Init.DeviceSize = 24;
  hospi2.Init.ChipSelectHighTime = 4;
  hospi2.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi2.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi2.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
  hospi2.Init.ClockPrescaler = 2;
  hospi2.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi2.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_ENABLE;
  hospi2.Init.ChipSelectBoundary = 23;
  hospi2.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_USED;
  hospi2.Init.MaxTran = 0;
  hospi2.Init.Refresh = 400;
  if (HAL_OSPI_Init(&hospi2) != HAL_OK)
  {
    Error_Handler();
  }
  sOspiManagerCfg.ClkPort = 2;
  sOspiManagerCfg.DQSPort = 2;
  sOspiManagerCfg.NCSPort = 2;
  sOspiManagerCfg.IOLowPort = HAL_OSPIM_IOPORT_2_LOW;
  sOspiManagerCfg.IOHighPort = HAL_OSPIM_IOPORT_2_HIGH;
  if (HAL_OSPIM_Config(&hospi2, &sOspiManagerCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  sHyperBusCfg.RWRecoveryTime = 3;
  sHyperBusCfg.AccessTime = 6;
  sHyperBusCfg.WriteZeroLatency = HAL_OSPI_LATENCY_ON_WRITE;
  sHyperBusCfg.LatencyMode = HAL_OSPI_FIXED_LATENCY;
  if (HAL_OSPI_HyperbusCfg(&hospi2, &sHyperBusCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI2_Init 2 */
  sCommand.AddressSpace = HAL_OSPI_MEMORY_ADDRESS_SPACE;
  sCommand.AddressSize  = HAL_OSPI_ADDRESS_32_BITS;
  sCommand.DQSMode      = HAL_OSPI_DQS_ENABLE;
  sCommand.Address      = 0;
  sCommand.NbData       = 1;

  if (HAL_OSPI_HyperbusCmd(&hospi2, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  sMemMappedCfg.TimeOutActivation = HAL_OSPI_TIMEOUT_COUNTER_DISABLE;

  if (HAL_OSPI_MemoryMapped(&hospi2, &sMemMappedCfg) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END OCTOSPI2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM23 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM23_Init(void)
{

  /* USER CODE BEGIN TIM23_Init 0 */

  /* USER CODE END TIM23_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM23_Init 1 */

  /* USER CODE END TIM23_Init 1 */
  htim23.Instance = TIM23;
  htim23.Init.Prescaler = 274;
  htim23.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim23.Init.Period = 4294967295;
  htim23.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim23.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim23) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim23, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim23) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim23, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim23, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim23, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim23, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM23_Init 2 */

  /* USER CODE END TIM23_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(US1_TR_GPIO_Port, US1_TR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LCD_BL_CTRL_Pin|US0_TR_Pin|US2_TR_Pin|RENDER_TIME_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MCU_ACTIVE_Pin|FRAME_RATE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_DISP_GPIO_Port, LCD_DISP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RLED_Pin|GLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VSYNC_FREQ_GPIO_Port, VSYNC_FREQ_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : US1_TR_Pin */
  GPIO_InitStruct.Pin = US1_TR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(US1_TR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_BL_CTRL_Pin US0_TR_Pin US2_TR_Pin */
  GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin|US0_TR_Pin|US2_TR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : RENDER_TIME_Pin */
  GPIO_InitStruct.Pin = RENDER_TIME_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(RENDER_TIME_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_ACTIVE_Pin FRAME_RATE_Pin */
  GPIO_InitStruct.Pin = MCU_ACTIVE_Pin|FRAME_RATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_DISP_Pin */
  GPIO_InitStruct.Pin = LCD_DISP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_DISP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RLED_Pin GLED_Pin */
  GPIO_InitStruct.Pin = RLED_Pin|GLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VSYNC_FREQ_Pin */
  GPIO_InitStruct.Pin = VSYNC_FREQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(VSYNC_FREQ_GPIO_Port, &GPIO_InitStruct);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_CLOSE);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_CLOSE);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PA0, SYSCFG_SWITCH_PA0_CLOSE);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//cpu의 DWT 레지스터 잠금 해제, 트레이스 활성화, 사이클 카운터 활성화 및 초기화
void DWT_Delay_Init(void)
{
  DWT->LAR = 0xC5ACCE55; // Unlock DWT registers

  // TRC 활성화
  CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;

  // 사이클 카운터 활성화
  DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; //0x00000001;

  // 카운터 초기화
  DWT->CYCCNT = 0;

  __ASM ("NOP");
  __ASM ("NOP");
  __ASM ("NOP");
}

//마이크로초 단위의 정밀한 딜레이를 위해 CPU 사이클 카운터 사
void DWT_Delay_us(uint32_t microseconds)
{
    uint32_t cycles = (SystemCoreClock / 1000000L) * microseconds;
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles);
}

void Start_All_PWM(void) {

    for (int i = 0; i < 2; i++)
        HAL_TIM_PWM_Start(servo_htim[i], servo_channel[i]);
}

void set_servo0_angle(uint16_t angle_pulse_value) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, angle_pulse_value);
}

void set_servo1_angle(uint16_t angle_pulse_value) {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, angle_pulse_value);
}

void Trigger_US0() {
    HAL_GPIO_WritePin(US0_TR_GPIO_Port, US0_TR_Pin, GPIO_PIN_SET);
    DWT_Delay_us(10); // Trig 펄스 길이
    HAL_GPIO_WritePin(US0_TR_GPIO_Port, US0_TR_Pin, GPIO_PIN_RESET);
}

void Trigger_US1() {
    HAL_GPIO_WritePin(US1_TR_GPIO_Port, US1_TR_Pin, GPIO_PIN_SET);
    DWT_Delay_us(10); // Trig 펄스 길이
    HAL_GPIO_WritePin(US1_TR_GPIO_Port, US1_TR_Pin, GPIO_PIN_RESET);
}

void Trigger_US2() {
    HAL_GPIO_WritePin(US2_TR_GPIO_Port, US2_TR_Pin, GPIO_PIN_SET);
    DWT_Delay_us(10); // Trig 펄스 길이
    HAL_GPIO_WritePin(US2_TR_GPIO_Port, US2_TR_Pin, GPIO_PIN_RESET);
}

//상승 에지 감지 시의 타이머 시간값과 하강 에지 감지시의 시간값 반환하여 거리 계산에 사
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) 
{
    if (htim->Instance == TIM23)
    {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) { // US0 (TIM23_CH4, PF9)
            if (g_echo_state_us0 == 0) { // 상승엣지 대기 중
                g_ic_rising_tick_us0 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
                g_echo_state_us0 = 1;     // 하강엣지 대기 상태로
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
                printf("US0 Rising Edge. Tick: %lu. State: %d\n", g_ic_rising_tick_us0, g_echo_state_us0);
            } else if (g_echo_state_us0 == 1) { // 하강엣지 대기 중
                g_ic_falling_tick_us0 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
                g_echo_state_us0 = 2;     // 측정 완료 상태로
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING); // 다음 측정을 위해
                printf("US0 Falling Edge. Tick: %lu. State: %d\n", g_ic_falling_tick_us0, g_echo_state_us0);
            }
        } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) { // US1 (TIM23_CH3, PF8)
            if (g_echo_state_us1 == 0) {
                g_ic_rising_tick_us1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
                g_echo_state_us1 = 1;
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
                printf("US1 Rising Edge. Tick: %lu. State: %d\n", g_ic_rising_tick_us1, g_echo_state_us1);
            } else if (g_echo_state_us1 == 1) {
                g_ic_falling_tick_us1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
                g_echo_state_us1 = 2;
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
                printf("US1 Falling Edge. Tick: %lu. State: %d\n", g_ic_falling_tick_us1, g_echo_state_us1);
            }
        } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) { // US2 (TIM23_CH1, PF6)
            if (g_echo_state_us2 == 0) {
                g_ic_rising_tick_us2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
                g_echo_state_us2 = 1;
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
                printf("US2 Rising Edge. Tick: %lu. State: %d\n", g_ic_rising_tick_us2, g_echo_state_us2);
            } else if (g_echo_state_us2 == 1) {
                g_ic_falling_tick_us2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
                g_echo_state_us2 = 2;
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
                printf("US2 Falling Edge. Tick: %lu. State: %d\n", g_ic_falling_tick_us2, g_echo_state_us2);
            }
        }
    }
}

void Start_US0_Measurement() {
    if (g_echo_state_us0 == 0 || g_echo_state_us0 == 2) {
        printf("Starting US0 Measurement. Current state: %d\n", g_echo_state_us0);
        g_echo_state_us0 = 0;
        g_ic_rising_tick_us0 = 0;
        g_ic_falling_tick_us0 = 0;

        __HAL_TIM_SET_CAPTUREPOLARITY(&htim23, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);

        Trigger_US0();
        printf("Triggered US0, waiting for echo (state should be 0)...\n");
    } else {
        printf("US0 measurement already in progress (state %d).\n", g_echo_state_us0);
    }
}

void Start_US1_Measurement() {
    if (g_echo_state_us1 == 0 || g_echo_state_us1 == 2) {
        printf("Starting US1 Measurement. Current state: %d\n", g_echo_state_us1);
        g_echo_state_us1 = 0;
        g_ic_rising_tick_us1 = 0;
        g_ic_falling_tick_us1 = 0;

        __HAL_TIM_SET_CAPTUREPOLARITY(&htim23, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);

        Trigger_US1();
        printf("Triggered US1, waiting for echo (state should be 0)...\n");
    } else {
        printf("US1 measurement already in progress (state %d).\n", g_echo_state_us1);
    }
}

void Start_US2_Measurement() {
    if (g_echo_state_us2 == 0 || g_echo_state_us2 == 2) {
        printf("Starting US2 Measurement. Current state: %d\n", g_echo_state_us2);
        g_echo_state_us2 = 0;
        g_ic_rising_tick_us2 = 0;
        g_ic_falling_tick_us2 = 0;

        __HAL_TIM_SET_CAPTUREPOLARITY(&htim23, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

        Trigger_US2();
        printf("Triggered US2, waiting for echo (state should be 0)...\n");
    } else {
        printf("US2 measurement already in progress (state %d).\n", g_echo_state_us2);
    }
}

float Get_US0_Distance_NonBlocking() {
    if (g_echo_state_us0 == 2) { // 측정 완료 상태일 때만 계산
        uint32_t duration_ticks = 0;
        uint32_t rising_tick = g_ic_rising_tick_us0;
        uint32_t falling_tick = g_ic_falling_tick_us0;
        uint32_t timer_period = htim23.Init.Period + 1;

        if (falling_tick >= rising_tick) {
            duration_ticks = falling_tick - rising_tick;
        } else { // 타이머 오버플로우 발생 시
            duration_ticks = (timer_period - rising_tick) + falling_tick;
        }

        float timer_clock_input_mhz = 275.0f;
        float tick_time_us = (float)(htim23.Init.Prescaler + 1) / timer_clock_input_mhz;
        float duration_us = duration_ticks * tick_time_us;
        float distance_cm = (duration_us * 0.0343f) / 2.0f; // 

        g_echo_state_us0 = 0;
        return distance_cm;
    }
    return MEASUREMENT_ERROR;
}

float Get_US1_Distance_NonBlocking() {
    if (g_echo_state_us1 == 2) {
        uint32_t duration_ticks = 0;
        uint32_t rising_tick = g_ic_rising_tick_us1;
        uint32_t falling_tick = g_ic_falling_tick_us1;
        uint32_t timer_period = htim23.Init.Period + 1;

        if (falling_tick >= rising_tick) {
            duration_ticks = falling_tick - rising_tick;
        } else { // 타이머 오버플로우 발생 시
            duration_ticks = (timer_period - rising_tick) + falling_tick;
        }

        float timer_clock_input_mhz = 275.0f;
        float tick_time_us = (float)(htim23.Init.Prescaler + 1) / timer_clock_input_mhz;
        float duration_us = duration_ticks * tick_time_us;
        float distance_cm = (duration_us * 0.0343f) / 2.0f;
        g_echo_state_us1 = 0;
        return distance_cm;
    }
    return MEASUREMENT_ERROR;
}

float Get_US2_Distance_NonBlocking() {
    if (g_echo_state_us2 == 2) {
        uint32_t duration_ticks = 0;
        uint32_t rising_tick = g_ic_rising_tick_us2;
        uint32_t falling_tick = g_ic_falling_tick_us2;
        uint32_t timer_period = htim23.Init.Period + 1;

        if (falling_tick >= rising_tick) {
            duration_ticks = falling_tick - rising_tick;
        } else { // 타이머 오버플로우 발생 시
            duration_ticks = (timer_period - rising_tick) + falling_tick;
        }

        float timer_clock_input_mhz = 275.0f;
        float tick_time_us = (float)(htim23.Init.Prescaler + 1) / timer_clock_input_mhz;
        float duration_us = duration_ticks * tick_time_us;
        float distance_cm = (duration_us * 0.0343f) / 2.0f; //음속 343m/s 가정
        g_echo_state_us2 = 0;
        return distance_cm;
    }
    return MEASUREMENT_ERROR;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartProximityTask */
/**
  * @brief  Function implementing the ProximityTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartProximityTask */

//사람의 접근을 2초 주기로 확인하고 오차 최소화를 위해 5번 측정 후 평균값 사용
void StartProximityTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
    ModelToProximityMsg_t rxMsg;
    ProximityToModelMsg_t txMsg;
    osStatus_t status;
    bool pollingActive = false;

    printf("ProximityTask: Started.\n");

    for (;;)
    {
        // 1. Model로부터 명령 수신 (논블로킹 또는 짧은 타임아웃으로)
        status = osMessageQueueGet(modelToProximityQueueHandle, &rxMsg, NULL, 0U); // Non-blocking

        if (status == osOK)
        {
            switch (rxMsg.command)
            {
                case PROXIMITY_CMD_START_POLLING_IMMEDIATE:
                    printf("ProximityTask: Received CMD_START_POLLING_IMMEDIATE.\n");
                    pollingActive = true;
                    // 혹시 이전 지연 타이머가 있었다면 취소하는 로직 (복잡도 증가)
                    break;
                case PROXIMITY_CMD_START_POLLING_DELAYED:
                    printf("ProximityTask: Received CMD_START_POLLING_DELAYED (delay: %lu ms).\n", rxMsg.delay_ms);
                    pollingActive = false; // 일단 현재 폴링은 중지 (또는 시작 안 함)
                    if (rxMsg.delay_ms > 0) {
                        osDelay(rxMsg.delay_ms); // 지정된 시간만큼 대기
                    }
                    pollingActive = true; // 대기 후 폴링 활성화
                    printf("ProximityTask: Delay finished, starting polling.\n");
                    break;
                case PROXIMITY_CMD_STOP_POLLING:
                    printf("ProximityTask: Received CMD_STOP_POLLING.\n");
                    pollingActive = false;
                    break;
            }
        }

    	float distance = MEASUREMENT_ERROR;
        if (pollingActive)
        {
      	    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
            float total_distance = 0.0f;
            int valid_measurements = 0;

            for (int i = 0; i < 5; i++) // 5번 측정
            {
              float current_distance = MEASUREMENT_ERROR;

              Start_US2_Measurement();
              osDelay(100);
              current_distance = Get_US2_Distance_NonBlocking();

              if (current_distance >= 5)
              {
                total_distance += current_distance;
                valid_measurements++;
                printf("SensorTask: Measurement%d: current_distance = %.2f\n",
              		  i + 1, current_distance);
              }

              if (i < 4)
              {
                osDelay(100);
              }
            } // end of 5 measurements loop

            if (valid_measurements > 1)
            {
              distance = total_distance / valid_measurements;
            }

            bool detected = 0;
            if (distance < DETECTION_DISTANCE_THRESHOLD_CM && distance > 0) {
            	detected = 1;
            }
            if (detected) {
                printf("ProximityTask: Person detected!\n");
                txMsg.event = PROXIMITY_EVENT_DETECTED;
                if (osMessageQueuePut(proximityToModelQueueHandle, &txMsg, 0U, 0U) != osOK) {
                    printf("ProximityTask: Failed to send DETECTED event to Model.\n");
                }
                pollingActive = false;
            }
        }

        // CPU 사용량 줄이기 위해 적절한 osDelay 사용
        // (pollingActive 일 때만 짧게, 아닐 때는 길게 또는 메시지 대기)
        if (!pollingActive && status != osOK) { // 명령도 없고 폴링도 안 할 때
            osDelay(500); // 긴 대기
        } else if (pollingActive && distance != MEASUREMENT_ERROR) {
            osDelay(PROXIMITY_POLLING_INTERVAL_MS); // 폴링 주기
        } else {
            osDelay(10); // 메시지 수신 시 빠른 반응을 위해 짧은 대기
        }
    }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the sensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */

//Model로부터 요청을 받아 특정 방향의 쓰레기 용량을 측정하여 큐로 반환함
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */
  CSensorRequestMessage_t receivedReqMsg; // <--- 헤더 파일에 정의된 이름으로 수정 (CSensorRequestMessage_t)
  CSensorResultMessage_t  resultMsgToSend;  // <--- 헤더 파일에 정의된 이름으로 수정 (CSensorResultMessage_t)
  osStatus_t status;

  printf("SensorTask: Started and waiting for measurement requests.\n");
  // DWT_Delay_Init(); // main에서 호출 가정

  /* Infinite loop */
  for(;;)
  {
    // 1. Model로부터 측정 요청 메시지 수신 (블로킹 대기)
    status = osMessageQueueGet(sensorRequestQueueHandle, &receivedReqMsg, NULL, osWaitForever);

    if (status == osOK)
    {
      printf("SensorTask: Received request - Type: %d\n", receivedReqMsg.requestType);

      resultMsgToSend.originalRequestType = receivedReqMsg.requestType;
      resultMsgToSend.success = false; // 기본적으로 실패로 설정
      resultMsgToSend.distance_cm = 0.0f; // 기본값
      float total_distance = 0.0f;
      int valid_measurements = 0;

      // 2. 요청 타입에 따라 센서 측정 수행 (5회 반복 및 평균)
      for (int i = 0; i < 5; i++) // 5번 측정
      {
        float current_distance = MEASUREMENT_ERROR; // MEASUREMENT_ERROR 정의 필요

        // 헤더 파일에 정의된 enum 값 사용
        if (receivedReqMsg.requestType == C_SENSOR_REQUEST_MEASURE_UP)
        {
          Start_US0_Measurement();
          osDelay(100);
          current_distance = Get_US0_Distance_NonBlocking();
        }
        else if (receivedReqMsg.requestType == C_SENSOR_REQUEST_MEASURE_RIGHT)
        {
          Start_US0_Measurement();
          osDelay(100);
          current_distance = Get_US0_Distance_NonBlocking();
        }
        else if (receivedReqMsg.requestType == C_SENSOR_REQUEST_MEASURE_DOWN)
        {
          Start_US1_Measurement();
          osDelay(100);
          current_distance = Get_US1_Distance_NonBlocking();
        }
        else if (receivedReqMsg.requestType == C_SENSOR_REQUEST_MEASURE_LEFT)
        {
          Start_US1_Measurement();
          osDelay(100);
          current_distance = Get_US1_Distance_NonBlocking();
        }
        else
        {
          printf("SensorTask: Unknown request type: %d\n", receivedReqMsg.requestType);
          break;
        }

        if (current_distance != MEASUREMENT_ERROR)
        {
          total_distance += current_distance;
          valid_measurements++;
          printf("SensorTask: Measurement%d: current_distance = %.2f\n",
        		  i + 1, current_distance);
      	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
        }
        else
        {
          printf("SensorTask: [Req %d] Measurement %d failed.\n", receivedReqMsg.requestType, i + 1);
        }

        if (i < 4)
        {
          osDelay(100);
        }
      } // end of 5 measurements loop

      // 3. 평균 계산 및 결과 메시지 준비
      if (valid_measurements > 1)
      {
        resultMsgToSend.distance_cm = total_distance / valid_measurements;
        resultMsgToSend.success = true;
        printf("SensorTask: Valid measurements: %d, Average distance: %.2f\n",
                               valid_measurements, resultMsgToSend.distance_cm);
      }
      else
      {
        printf("SensorTask: [Req %d] All 3 measurements failed.\n", receivedReqMsg.requestType);
      }

      // 4. Model(GUI Task)로 측정 결과 메시지 전송
      status = osMessageQueuePut(sensorResultQueueHandle, &resultMsgToSend, 0U, 0U);
      if (status != osOK)
      {
        printf("SensorTask: Failed to send result message for request type %d. HAL Status: %d\n",
                       receivedReqMsg.requestType, status);
        // Error_Handler(); // CubeMX 생성 함수 사용 또는 다른 에러 처리
      }
      else
      {
         printf("SensorTask: Result message sent for request type %d.\n", receivedReqMsg.requestType);
      }
    } // end of if (status == osOK) for Get
    else
    {
      printf("SensorTask: Error getting message from sensorRequestQueue. Status: %d\n", status);
      osDelay(100);
    }
  } // end of infinite loop for(;;)
  /* USER CODE END StartSensorTask */
}

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x24000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x70000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.Size = MPU_REGION_SIZE_8MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER3;
  MPU_InitStruct.BaseAddress = 0x90000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER4;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	osDelay(1);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
