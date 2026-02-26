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
#include "cmsis_os.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "bms.h"
#include "can_driver.h"
#include "can_messages.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#if !defined(BMS_MODEL_10S) && !defined(BMS_MODEL_14S)
#error "Must define BMS_MODEL_10S or BMS_MODEL_14S"
#endif

#define MIN_PACK_VOLTAGE_MV  (25000u)    // 2.5V per cell * 10 cells
#define MAX_PACK_VOLTAGE_MV  (42000u)    // 4.2V
#define DIFF_PACK_VOLTAGE_MV (MAX_PACK_VOLTAGE_MV - MIN_PACK_VOLTAGE_MV)

#define APPLICATION_ADDRESS (0x08004000u)    // Defined in linker script FLASH ORIGIN
#define FLASH_LENGTH        (0x0001C000u)    // Defined in linker script FLASH LENGTH
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

CRC_HandleTypeDef hcrc;

DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

/* Definitions for defaultTask */
osThreadId_t         defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name       = "defaultTask",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};
/* Definitions for BMSMonitor */
osThreadId_t         BMSMonitorHandle;
const osThreadAttr_t BMSMonitor_attributes = {
    .name       = "BMSMonitor",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};
/* Definitions for FuelGaugeMonitor */
osThreadId_t         FuelGaugeMonitorHandle;
const osThreadAttr_t FuelGaugeMonitor_attributes = {
    .name       = "FuelGaugeMonitor",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityLow,
};
/* Definitions for CANMonitor */
osThreadId_t         CANMonitorHandle;
const osThreadAttr_t CANMonitor_attributes = {
    .name       = "CANMonitor",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityLow,
};
/* Definitions for bms_task */
osThreadId_t         bms_taskHandle;
const osThreadAttr_t bms_task_attributes = {
    .name       = "bms_task",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};
/* Definitions for CANQueue */
osMessageQueueId_t         CANQueueHandle;
const osMessageQueueAttr_t CANQueue_attributes = {
    .name = "CANQueue"};
/* Definitions for mutex_01 */
osMutexId_t         mutex_01Handle;
const osMutexAttr_t mutex_01_attributes = {
    .name = "mutex_01"};
/* Definitions for semaphore_1 */
osSemaphoreId_t         semaphore_1Handle;
const osSemaphoreAttr_t semaphore_1_attributes = {
    .name = "semaphore_1"};
/* USER CODE BEGIN PV */

char opening_msg[] = "\r\n\r\nEKTELI BMS, version 1.1\r\n";

const uint16_t version = 110;    // e.g: ver 1.0.0 -> 100, ver 1.1.0 -> 110

// for debugging purposes
volatile uint8_t resetear_bms = 0;    // Set to 1 to reset the BMS by sending the reset command in the main loop

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void        SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_DAC1_Init(void);
static void MX_RTC_Init(void);
static void MX_CRC_Init(void);
void        StartDefaultTask(void *argument);
void        bms_monitor(void *argument);
void        fuel_gauge_monitor(void *argument);
void        can_monitor(void *argument);
void        bms_main_task(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief Transmits the firmware version and CRC value over CAN bus.
 *
 * This function prepares a 6-byte buffer containing the firmware version
 * (2 bytes) and the 32-bit CRC value (4 bytes) calculated from the flash
 * memory region starting at APPLICATION_ADDRESS. The buffer is then sent
 * via CAN using the specified CAN message ID.
 *
 * Buffer layout:
 *   buffer[0-1]: Firmware version (big-endian)
 *   buffer[2-5]: CRC value (big-endian)
 *
 * @note The CRC is computed using HAL_CRC_Calculate with FLASH_LENGTH.
 */
void transmit_fw_version(void)
{
    uint8_t buffer[6];

    /* Get Firmware Version */
    buffer[0] = (version >> 8) & 0xFF;
    buffer[1] = version & 0xFF;

    /* Get 32bit CRC Flash Memory value */
    uint32_t crc_value = HAL_CRC_Calculate(&hcrc, (uint32_t *)APPLICATION_ADDRESS, FLASH_LENGTH / 4);

    buffer[2] = (crc_value >> 24) & 0xFF;
    buffer[3] = (crc_value >> 16) & 0xFF;
    buffer[4] = (crc_value >> 8) & 0xFF;
    buffer[5] = crc_value & 0xFF;

    /* Transmit on CAN */
    can_msg_transmit(CAN_ID_BMS_FW_VER, buffer, 6, 100u);
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
    (void)HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_CAN1_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_DAC1_Init();
    MX_RTC_Init();
    MX_CRC_Init();
    /* USER CODE BEGIN 2 */

    // Set DAC output to 3.3V to signal main app is started
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4095);

    HAL_CAN_Start(&hcan1);

    can_msg_transmit(CAN_ID_BMS, (uint8_t *)opening_msg, strlen(opening_msg), 100u);

#if 0
    if (version == 2)
    {
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0);
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0);
    }

    while (1)
    {
        can_decode_cmd();
        HAL_Delay(10);
    }
#endif

    transmit_fw_version();

    subcommands(ADDR_DEVICE_NUMBER, 0, 0);
    direct_commands(ADDR_BATTERY_STATUS, 0, 0);    // read
    direct_commands(ADDR_ALARM_RAW_STATUS, 0, 0);
    HAL_Delay(60);

    bms_otp_status_t otp_status = bms_otp_check();

    bms_init();    // Configure all of the BQ769x2 register settings

    HAL_Delay(10);
    command_subcommands(ADDR_FET_ENABLE);    // Enable the CHG and DSG FETs
    HAL_Delay(10);
    command_subcommands(ADDR_SLEEP_DISABLE);
    /* USER CODE END 2 */

    /* Init scheduler */
    osKernelInitialize();
    /* Create the mutex(es) */
    /* creation of mutex_01 */
    mutex_01Handle = osMutexNew(&mutex_01_attributes);

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* Create the semaphores(s) */
    /* creation of semaphore_1 */
    semaphore_1Handle = osSemaphoreNew(1, 1, &semaphore_1_attributes);

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* Create the queue(s) */
    /* creation of CANQueue */
    CANQueueHandle = osMessageQueueNew(32, sizeof(can_message_t), &CANQueue_attributes);

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of defaultTask */
    defaultTaskHandle = osThreadNew(StartDefaultTask, (void *)(uintptr_t)otp_status, &defaultTask_attributes);

    /* creation of BMSMonitor */
    BMSMonitorHandle = osThreadNew(bms_monitor, NULL, &BMSMonitor_attributes);

    /* creation of FuelGaugeMonitor */
    FuelGaugeMonitorHandle = osThreadNew(fuel_gauge_monitor, NULL, &FuelGaugeMonitor_attributes);

    /* creation of CANMonitor */
    CANMonitorHandle = osThreadNew(can_monitor, NULL, &CANMonitor_attributes);

    /* creation of bms_task */
    bms_taskHandle = osThreadNew(bms_main_task, NULL, &bms_task_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
#if 0
    if (otp_status != BMS_OTP_OK)
    {
        osThreadSuspend(defaultTaskHandle);
    }
#endif
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

    /** Configure the main internal regulator output voltage
     */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.LSIState            = RCC_LSI_ON;
    RCC_OscInitStruct.MSIState            = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange       = RCC_MSIRANGE_6;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM            = 1;
    RCC_OscInitStruct.PLL.PLLN            = 40;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void)
{
    /* USER CODE BEGIN CAN1_Init 0 */

    /* USER CODE END CAN1_Init 0 */

    /* USER CODE BEGIN CAN1_Init 1 */

    /* USER CODE END CAN1_Init 1 */
    hcan1.Instance                  = CAN1;
    hcan1.Init.Prescaler            = 8;
    hcan1.Init.Mode                 = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth        = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1             = CAN_BS1_14TQ;
    hcan1.Init.TimeSeg2             = CAN_BS2_5TQ;
    hcan1.Init.TimeTriggeredMode    = DISABLE;
    hcan1.Init.AutoBusOff           = ENABLE;
    hcan1.Init.AutoWakeUp           = ENABLE;
    hcan1.Init.AutoRetransmission   = ENABLE;
    hcan1.Init.ReceiveFifoLocked    = DISABLE;
    hcan1.Init.TransmitFifoPriority = ENABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN CAN1_Init 2 */

    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh         = 0x0000;
    sFilterConfig.FilterIdLow          = 0x0000;
    sFilterConfig.FilterMaskIdHigh     = 0x0000;
    sFilterConfig.FilterMaskIdLow      = 0x0000;
    sFilterConfig.SlaveStartFilterBank = 14;
    sFilterConfig.FilterBank           = 0;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation     = CAN_FILTER_ENABLE;

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
        /* Filter configuration Error */
        Error_Handler();
    }

    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    /* USER CODE END CAN1_Init 2 */
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
    hcrc.Instance                     = CRC;
    hcrc.Init.DefaultPolynomialUse    = DEFAULT_POLYNOMIAL_ENABLE;
    hcrc.Init.DefaultInitValueUse     = DEFAULT_INIT_VALUE_ENABLE;
    hcrc.Init.InputDataInversionMode  = CRC_INPUTDATA_INVERSION_NONE;
    hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
    hcrc.InputDataFormat              = CRC_INPUTDATA_FORMAT_WORDS;
    if (HAL_CRC_Init(&hcrc) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN CRC_Init 2 */

    /* USER CODE END CRC_Init 2 */
}

/**
 * @brief DAC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC1_Init(void)
{
    /* USER CODE BEGIN DAC1_Init 0 */

    /* USER CODE END DAC1_Init 0 */

    DAC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN DAC1_Init 1 */

    /* USER CODE END DAC1_Init 1 */

    /** DAC Initialization
     */
    hdac1.Instance = DAC1;
    if (HAL_DAC_Init(&hdac1) != HAL_OK)
    {
        Error_Handler();
    }

    /** DAC channel OUT1 config
     */
    sConfig.DAC_SampleAndHold           = DAC_SAMPLEANDHOLD_DISABLE;
    sConfig.DAC_Trigger                 = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer            = DAC_OUTPUTBUFFER_ENABLE;
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
    sConfig.DAC_UserTrimming            = DAC_TRIMMING_FACTORY;
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN DAC1_Init 2 */

    /* USER CODE END DAC1_Init 2 */
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
    hi2c1.Instance              = I2C1;
    hi2c1.Init.Timing           = 0x10D19CE4;
    hi2c1.Init.OwnAddress1      = 0;
    hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2      = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
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
    hrtc.Instance            = RTC;
    hrtc.Init.HourFormat     = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv   = 127;
    hrtc.Init.SynchPrediv    = 255;
    hrtc.Init.OutPut         = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutRemap    = RTC_OUTPUT_REMAP_NONE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
    if (HAL_RTC_Init(&hrtc) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN RTC_Init 2 */

    /* USER CODE END RTC_Init 2 */
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
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 7;
    hspi1.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */
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
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, RST_SHUT_Pin | DFETOFF_Pin | TP5_Pin | CAN_SILENT_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin : BAT_MON_ALERT_Pin */
    GPIO_InitStruct.Pin  = BAT_MON_ALERT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BAT_MON_ALERT_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PC15 */
    GPIO_InitStruct.Pin  = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : RST_SHUT_Pin */
    GPIO_InitStruct.Pin   = RST_SHUT_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RST_SHUT_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : DDSG_Pin DCHG_Pin */
    GPIO_InitStruct.Pin  = DDSG_Pin | DCHG_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : DFETOFF_Pin TP5_Pin CAN_SILENT_Pin */
    GPIO_InitStruct.Pin   = DFETOFF_Pin | TP5_Pin | CAN_SILENT_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : SPI_CS_Pin */
    GPIO_InitStruct.Pin   = SPI_CS_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PB1 PB4 */
    GPIO_InitStruct.Pin  = GPIO_PIN_1 | GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : PA8 PA15 */
    GPIO_InitStruct.Pin  = GPIO_PIN_8 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : FUEL_GAUGE_ALERT_Pin */
    GPIO_InitStruct.Pin  = FUEL_GAUGE_ALERT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(FUEL_GAUGE_ALERT_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PH3 */
    GPIO_InitStruct.Pin  = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */

    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
    /* USER CODE BEGIN 5 */
    bms_otp_status_t otp_status = (bms_otp_status_t)(uintptr_t)argument;

    uint32_t delay_ms;

    switch (otp_status)
    {
        case BMS_OTP_OK:
            delay_ms = 500u;
            break;

        case BMS_OTP_NOT_PROGRAMMED:
            delay_ms = 250u;
            break;

        case BMS_OTP_BQ_NOT_DETECTED:
            delay_ms = 167u;
            break;

        case BMS_OTP_CONDITIONS_NOT_MET:
            delay_ms = 125u;
            break;

        case BMS_OTP_FAILED:
            delay_ms = 100u;
            break;

        case BMS_OTP_ERROR:
            delay_ms = 83u;
            break;

        default:
            delay_ms = 50u;
            break;
    }

    /* Infinite loop */
    for (;;)
    {
        HAL_GPIO_TogglePin(TP5_GPIO_Port, TP5_Pin);
        osDelay(pdMS_TO_TICKS(delay_ms));
    }
    /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_bms_monitor */
/**
 * @brief Function implementing the BMSMonitor thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_bms_monitor */
void bms_monitor(void *argument)
{
    /* USER CODE BEGIN bms_monitor */
    UNUSED(argument);

    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
    /* USER CODE END bms_monitor */
}

/* USER CODE BEGIN Header_fuel_gauge_monitor */
/**
 * @brief Function implementing the FuelGaugeMonitor thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_fuel_gauge_monitor */
void fuel_gauge_monitor(void *argument)
{
    /* USER CODE BEGIN fuel_gauge_monitor */
    UNUSED(argument);

    uint8_t data[8] = {0};

    data[0] = 0x7E;
    data[1] = 0x7F;

    HAL_I2C_IsDeviceReady(&hi2c1, 0xAB, 3, 100);
    HAL_I2C_Master_Transmit(&hi2c1, 0xAA, data, 2, 100);
    HAL_I2C_Master_Receive(&hi2c1, 0xAB, &data[3], 2, 100);
    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
    /* USER CODE END fuel_gauge_monitor */
}

/* USER CODE BEGIN Header_can_monitor */
/**
 * @brief Function implementing the CANMonitor thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_can_monitor */
void can_monitor(void *argument)
{
    /* USER CODE BEGIN can_monitor */
    UNUSED(argument);

    can_message_t msg;
    for (;;)
    {
        if (osMessageQueueGet(CANQueueHandle, &msg, NULL, osWaitForever) == osOK)
        {
            can_decode_cmd(&msg);
        }
        osDelay(pdMS_TO_TICKS(10));
    }
    /* USER CODE END can_monitor */
}

/* USER CODE BEGIN Header_bms_main_task */
/**
 * @brief Function implementing the bms_task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_bms_main_task */
void bms_main_task(void *argument)
{
    /* USER CODE BEGIN bms_main_task */
    UNUSED(argument);

    uint8_t buffer[8];
    float   temperature[3] = {0.0f};

    /* Infinite loop */
    for (;;)
    {
        /* Send Battery Status */
        // SoC and SoH come from the Fuel Gauge IC. for now estimating SoC based on voltage,
        // SoH is set to 100%
        uint16_t voltage = bq769x2_read_voltage(ADDR_STACK_VOLTAGE);
        if (voltage < MIN_PACK_VOLTAGE_MV)
        {
            voltage = MIN_PACK_VOLTAGE_MV;
        }

        int16_t current = bq769x2_read_current() / 10;    // mA -> A -> CAN doc units: (val mA * (1 A / 1000 mA) * (100 units / 1 A)) = val / 10

        temperature[0] = bq769x2_read_temperature(0x70);    // TS1Temperature
        temperature[1] = bq769x2_read_temperature(0x74);    // TS3Temperature
        // TS2 not used, third temp sensor is on Fuel Gauge IC

        float temp_avg_f = (temperature[0] + temperature[1]) / 2.0f;
        if (temp_avg_f < -40.0f)
        {
            temp_avg_f = -40.0f;
        }
        if (temp_avg_f > 215.0f)
        {
            temp_avg_f = 215.0f;
        }

        uint8_t temp_avg = (uint8_t)(temp_avg_f + 40.0f);

        uint8_t soc = (voltage - MIN_PACK_VOLTAGE_MV) * 100 / DIFF_PACK_VOLTAGE_MV;
        uint8_t soh = 100u;

        buffer[0] = soc;
        buffer[1] = soh;
        buffer[2] = (voltage >> 8) & 0xFF;
        buffer[3] = voltage & 0xFF;
        buffer[4] = (current >> 8) & 0xFF;
        buffer[5] = current & 0xFF;
        buffer[6] = temp_avg;
        can_msg_transmit(CAN_ID_BATTERY_STATUS, (uint8_t *)&buffer, 8, 100u);

        /* Send BMS Safety & Alarm Flags */
        bq769x2_read_safety_status();
        bq769x2_read_pf_status();
        bq769x2_read_cell_voltages();

        uint16_t min_voltage = get_smallest_cell_voltage();
        uint16_t max_voltage = get_largest_cell_voltage();
        float    max_temp    = temperature[0] > temperature[1] ? temperature[0] : temperature[1];
        if (max_temp < -40.0f)
        {
            max_temp = -40.0f;
        }
        if (max_temp > 215.0f)
        {
            max_temp = 215.0f;
        }

        uint16_t alarm_bits = bq769x2_read_alarm_status();
        uint8_t  fuse_ok;
        if (alarm_bits & 0x0008)
        {
            fuse_ok = 0;    // blown
        }
        else
        {
            fuse_ok = 1;    // ok
        }

        buffer[0] = bms_get_faults();
        buffer[1] = (min_voltage >> 8) & 0xFF;
        buffer[2] = min_voltage & 0xFF;
        buffer[3] = (max_voltage >> 8) & 0xFF;
        buffer[4] = max_voltage & 0xFF;
        buffer[5] = (uint8_t)(max_temp + 40.0f);
        buffer[6] = fuse_ok;
        can_msg_transmit(CAN_ID_BMS_SAFETY, (uint8_t *)&buffer, 8, 100u);

        /* Send BMS Operating Status */
        int16_t max_discharge_current = (int16_t)(10.0f * 100.0f);    // set 10A max discharge current for now
        int16_t max_charge_current    = (int16_t)(3.0f * 100.0f);     // set 3A max charge current for now

        buffer[0] = get_balancing_status();
        buffer[1] = (max_discharge_current >> 8) & 0xFF;
        buffer[2] = max_discharge_current & 0xFF;
        buffer[3] = (max_charge_current >> 8) & 0xFF;
        buffer[4] = max_charge_current & 0xFF;

        bq769x2_read_fet_status();

        buffer[5] = get_charging_status();
        buffer[6] = get_bms_status();
        buffer[7] = get_fet_status();
        can_msg_transmit(CAN_ID_BMS_OPERATION, (uint8_t *)&buffer, 8, 100u);

        if (resetear_bms == 1u)
        {
            command_subcommands(ADDR_RESET);
            HAL_Delay(100);
            bms_init();
        }

        osDelay(pdMS_TO_TICKS(500));
    }
    /* USER CODE END bms_main_task */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM7 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM7)
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
__NO_RETURN void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
__NO_RETURN __attribute__((naked)) void assert_failed(const uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    UNUSED(file);
    UNUSED(line);

    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
