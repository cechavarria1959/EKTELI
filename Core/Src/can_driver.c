/*
 * can_driver.c
 *
 *  Created on: Nov 13, 2025
 *      Author: CesarO
 */

/**
 * @file    can_driver.c
 * @brief   CAN driver implementation.
 * @details
 *
 * @author  CesarO
 * @date    2025-11-13
 *
 * @copyright Copyright (c) 2025 Ekteli.
 */

/* Private includes ----------------------------------------------------------*/
#include <stdint.h>
#include "can_driver.h"
#include "can_messages.h"
#include "main.h"


/* Private macros ------------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/


/* Exported types and variables ----------------------------------------------*/


/* Private (static) variables ------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/


/* Public user code ----------------------------------------------------------*/
void can_transmit_status(uint8_t soc, uint8_t soh, uint16_t voltage, int16_t current, int8_t temp_avg)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t             data[8] = {0};
    uint32_t            mailbox;

    tx_header.StdId              = CAN_ID_BATTERY_STATUS;
    tx_header.RTR                = CAN_RTR_DATA;
    tx_header.IDE                = CAN_ID_STD;
    tx_header.DLC                = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    data[0] = soc;
    data[1] = soh;
    data[2] = (uint8_t)(voltage >> 8);
    data[3] = (uint8_t)(voltage & 0xFF);
    data[4] = (uint8_t)(current >> 8);
    data[5] = (uint8_t)(current & 0xFF);
    data[6] = (uint8_t)temp_avg;
    data[7] = 0x00;

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &mailbox);
}


/* Private user code ---------------------------------------------------------*/
