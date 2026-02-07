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
#include <string.h>
#include "can_driver.h"
#include "can_messages.h"
#include "main.h"


/* Private macros ------------------------------------------------------------*/
#define FW_UPDATE_BYTE_SEQUENCE_1 (0x5555AAAA)
#define FW_UPDATE_BYTE_SEQUENCE_2 (0xAAAA5555)


/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    GET_VOLTAGE,
    GET_CURRENT,
    GET_SOH,
    GET_SOC,
    FW_UPDATE
} can_command_t;


/* Exported types and variables ----------------------------------------------*/


/* Private (static) variables ------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
HAL_StatusTypeDef can_msg_ack(uint32_t can_id, uint32_t timeout);


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

HAL_StatusTypeDef can_msg_transmit(uint32_t can_id, uint8_t *pdata, uint32_t length, uint32_t timeout)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t             tx_data[8];
    uint32_t            tx_mailbox;

    uint32_t tickstart = HAL_GetTick();
    uint32_t tx_count  = length;

    while (tx_count > 0u)
    {
        /* Wait until tx buffer is available */
        while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0u)
        {
            /* Check for the Timeout */
            if (timeout != HAL_MAX_DELAY)
            {
                if (((HAL_GetTick() - tickstart) > timeout) || (timeout == 0u))
                {
                    return HAL_TIMEOUT;
                }
            }
        }

        uint8_t bytes_to_copy = (tx_count >= 8u) ? 8u : (uint8_t)tx_count;
        memset(tx_data, 0, sizeof(tx_data));

        for (uint8_t i = 0u; i < bytes_to_copy; i++)
        {
            tx_data[i] = *pdata++;
        }

        tx_count -= bytes_to_copy;

        tx_header.StdId = can_id;
        tx_header.IDE   = CAN_ID_STD;
        tx_header.RTR   = CAN_RTR_DATA;
        tx_header.DLC   = bytes_to_copy;

        HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
    }

    return HAL_OK;
}

/**
 * @brief Decodes and processes incoming CAN bus commands.
 *
 * This function handles CAN messages received by the BMS. It distinguishes between
 * data frames and remote transmission request (RTR) frames, and processes each
 * message according to its standard identifier (StdId).
 *
 * For RTR frames:
 *   - CAN_ID_BMS_GET_PROTECTIONS: Responds with current protection settings.
 *   - CAN_ID_BMS_GET_FAULTS: Responds with current fault status.
 *   - CAN_ID_BMS_GET_FW_VER: Responds with the current firmware version.
 *
 * @param msg Pointer to the received CAN message structure.
 */
void can_decode_cmd(can_message_t *msg)
{
    if (msg->header.RTR == CAN_RTR_DATA)
    {
        can_command_t cmd;
        UNUSED(cmd);

        switch (msg->header.StdId)
        {
            case CAN_ID_BMS_STATE:
                cmd = (can_command_t)msg->data[0];
                can_msg_ack(CAN_ID_BMS_STATE, 100u);
                break;

            case CAN_ID_BMS_BALANCE:
                cmd = (can_command_t)msg->data[0];
                can_msg_ack(CAN_ID_BMS_BALANCE, 100u);
                break;

            case CAN_ID_BMS_SET_PROTECTIONS:
                can_msg_ack(CAN_ID_BMS_SET_PROTECTIONS, 100u);
                break;

            case CAN_ID_BMS_RESET:
                can_msg_ack(CAN_ID_BMS_RESET, 100u);
                CommandSubcommands(ADDR_RESET);
                osDelay(pdMS_TO_TICKS(10u));
                HAL_NVIC_SystemReset();
                break;

            case CAN_ID_FW_UPDATE:
                if (msg->data[0] == CAN_ID_BMS)
                {
                    can_msg_ack(CAN_ID_FW_UPDATE, 100u);
                    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, FW_UPDATE_BYTE_SEQUENCE_1);
                    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, FW_UPDATE_BYTE_SEQUENCE_2);
                    HAL_Delay(1);
                    HAL_NVIC_SystemReset();
                }
                break;

            default:
                break;
        }
    }
    else    // RTR Request frame
    {
        switch (msg->header.StdId)
        {
            case CAN_ID_BMS_GET_PROTECTIONS:
                can_msg_ack(CAN_ID_BMS_PROTECTIONS, 100u);
                break;

            case CAN_ID_BMS_GET_FAULTS:
                can_msg_ack(CAN_ID_BMS_FAULTS, 100u);
                break;

            case CAN_ID_BMS_GET_FW_VER:
                transmit_fw_version();
                break;

            default:
                break;
        }
    }
}

HAL_StatusTypeDef can_msg_ack(uint32_t can_id, uint32_t timeout)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t             tx_data[8];
    uint32_t            tx_mailbox;

    uint32_t tickstart = HAL_GetTick();

    /* Wait until tx buffer is available */
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0u)
    {
        /* Check for the Timeout */
        if (timeout != HAL_MAX_DELAY)
        {
            if (((HAL_GetTick() - tickstart) > timeout) || (timeout == 0u))
            {
                return HAL_TIMEOUT;
            }
        }
    }

    tx_header.StdId = can_id;
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = 0u;

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);

    return HAL_OK;
}


/* Private user code ---------------------------------------------------------*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    can_message_t msg;

    uint8_t msgcount = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0);

    while (msgcount-- > 0)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &msg.header, msg.data);
        osMessageQueuePut(CANQueueHandle, &msg, 0, 0);
    }

    __NOP();
}
