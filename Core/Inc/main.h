/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_MAIN_H_
#define INC_MAIN_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

    /* Private includes ----------------------------------------------------------*/
    /* USER CODE BEGIN Includes */

    /* USER CODE END Includes */

    /* Exported types ------------------------------------------------------------*/
    /* USER CODE BEGIN ET */

    /* USER CODE END ET */

    /* Exported constants --------------------------------------------------------*/
    /* USER CODE BEGIN EC */

    /* USER CODE END EC */

    /* Exported macro ------------------------------------------------------------*/
    /* USER CODE BEGIN EM */

    /* USER CODE END EM */

    /* Exported functions prototypes ---------------------------------------------*/
    void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BAT_MON_ALERT_Pin          GPIO_PIN_14
#define BAT_MON_ALERT_GPIO_Port    GPIOC
#define RST_SHUT_Pin               GPIO_PIN_0
#define RST_SHUT_GPIO_Port         GPIOA
#define DDSG_Pin                   GPIO_PIN_1
#define DDSG_GPIO_Port             GPIOA
#define DCHG_Pin                   GPIO_PIN_2
#define DCHG_GPIO_Port             GPIOA
#define DFETOFF_Pin                GPIO_PIN_3
#define DFETOFF_GPIO_Port          GPIOA
#define TP4_DAC_Pin                GPIO_PIN_4
#define TP4_DAC_GPIO_Port          GPIOA
#define SPI_SCK_Pin                GPIO_PIN_5
#define SPI_SCK_GPIO_Port          GPIOA
#define SPI_MISO_Pin               GPIO_PIN_6
#define SPI_MISO_GPIO_Port         GPIOA
#define SPI_MOSI_Pin               GPIO_PIN_7
#define SPI_MOSI_GPIO_Port         GPIOA
#define SPI_CS_Pin                 GPIO_PIN_0
#define SPI_CS_GPIO_Port           GPIOB
#define TP5_Pin                    GPIO_PIN_9
#define TP5_GPIO_Port              GPIOA
#define CAN_SILENT_Pin             GPIO_PIN_10
#define CAN_SILENT_GPIO_Port       GPIOA
#define FUEL_GAUGE_ALERT_Pin       GPIO_PIN_5
#define FUEL_GAUGE_ALERT_GPIO_Port GPIOB

    /* USER CODE BEGIN Private defines */

    /* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* INC_MAIN_H_ */
