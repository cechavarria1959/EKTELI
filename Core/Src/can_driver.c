/**
 * @file    can_driver.c
 * @brief   CAN driver implementation.
 * @details Provides functions for CAN message transmission, reception,
 *          and command decoding for battery management system.
 *
 * @author  CesarO
 * @date    2025-11-13
 *
 * @copyright Copyright (c) 2025 Ekteli.
 */

/* Private includes ----------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include "bms.h"
#include "can_driver.h"
#include "can_messages.h"
#include "main.h"


/* Private macros ------------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    GET_VOLTAGE,
    GET_CURRENT,
    GET_SOH,
    GET_SOC,
    FW_UPDATE,

    BMS_MODE_INACTIVE = 0,
    BMS_MODE_ACTIVE,
    BMS_MODE_SLEEP,
    BMS_MODE_SHUTDOWN,
    SET_NO_BALANCE = 0
} can_command_t;


/* Exported types and variables ----------------------------------------------*/


/* Private (static) variables ------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
HAL_StatusTypeDef can_msg_ack(uint32_t can_id, uint32_t timeout);


/* Public user code ----------------------------------------------------------*/
/**
 * @brief Transmits arbitrary CAN message with timeout.
 *
 * Sends a CAN message with the specified CAN ID and data payload. Handles
 * splitting the data into multiple CAN frames if the payload exceeds 8 bytes.
 * Waits for a free transmit mailbox and supports timeout for transmission.
 *
 * @param can_id   Standard CAN identifier for the message.
 * @param pdata    Pointer to data buffer to transmit.
 * @param length   Length of data buffer in bytes.
 * @param timeout  Timeout duration in milliseconds. Use HAL_MAX_DELAY for
 *                 infinite wait.
 * @return HAL_OK on success, HAL_TIMEOUT if transmission timed out.
 */
HAL_StatusTypeDef can_msg_transmit(uint32_t can_id, uint8_t *pdata, uint32_t length, uint32_t timeout)
{
    CAN_TxHeaderTypeDef tx_header = {0};
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
                /*Change BMS Mode: Inactive: 0, Active: 1, Sleep: 2, Shutdown: 3*/
                /* Inactive: sleep, reduced power consumption. bms allowed to
                             enter sleep if current is < Power:Sleep:Sleep Current
                 * Active: normal operation
                 * Sleep: deepsleep, only wake up on CAN message
                 * Shutdown: shutdown lowest power state, for shipment or long-term storage
                 */
                switch (cmd)
                {
                    case BMS_MODE_INACTIVE:
                        /*out if current > Power:Sleep:Wake Comparator Current or other means*/
                        command_subcommands(ADDR_SLEEP_ENABLE);
                        break;

                    case BMS_MODE_ACTIVE:
                        command_subcommands(ADDR_SLEEP_DISABLE);
                        break;

                    case BMS_MODE_SLEEP:
                        enter_sleep_mode();
                        break;

                    case BMS_MODE_SHUTDOWN:
                        bms_reset_shutdown(GPIO_PIN_SET);
                        /* After 1s device should enter shutdown mode */
                        break;

                    default:
                        break;
                }
                break;

            case CAN_ID_BMS_BALANCE:
                cmd = (can_command_t)msg->data[0];
                can_msg_ack(CAN_ID_BMS_BALANCE, 100u);
                /*Change Balancing Op.	Off: 0, On: 1*/
                if(cmd == SET_NO_BALANCE)
                {
                    subcommands(ADDR_CB_ACTIVE_CELLS, 0, 2);
                }
                else
                {
                    subcommands(ADDR_CB_SET_LVL1, 3276, 2); //to check
                }
                break;

            case CAN_ID_BMS_SET_PROTECTIONS:
                can_msg_ack(CAN_ID_BMS_SET_PROTECTIONS, 100u);
                /*  0	OV Protection	Set OV in mV
                    2	UV Protection	Set UV in mV
                    4	OT Protection	Set OT in degC
                    5	OC Protection	Set OC in A
                    */
                break;

            case CAN_ID_BMS_RESET:
                can_msg_ack(CAN_ID_BMS_RESET, 100u);
                command_subcommands(ADDR_RESET);
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
                    /*  0	OV Protection	Set OV in mV
                        2	UV Protection	Set UV in mV
                        4	OT Protection	Set OT in degC
                        5	OC Protection	Set OC in A
                        */
                break;

            case CAN_ID_BMS_GET_FAULTS:
                can_msg_ack(CAN_ID_BMS_FAULTS, 100u);
                /*  0	Index	0=Error 1, 1=Error 2, 2=Error 3
                    1	Error index	Error code
                    2	Timestamp Error index	Time when Error 1 recorded
                    */
                /* TODO: Send three frames, one frame per error */
                break;

            case CAN_ID_BMS_GET_FW_VER:
                transmit_fw_version();
                break;

            default:
                break;
        }
    }
}

/**
 * @brief Sends a CAN message acknowledgment with the specified CAN ID.
 *
 * This function waits until a CAN transmit mailbox is available, then sends
 * an empty CAN message (DLC = 0) with the given standard CAN ID. If the
 * mailbox is not available within the specified timeout, the function returns
 * HAL_TIMEOUT.
 *
 * @param can_id   Standard CAN identifier to use for the acknowledgment.
 * @param timeout  Timeout duration in milliseconds. Use HAL_MAX_DELAY for
 *                 indefinite wait.
 * @return HAL_OK on success, HAL_TIMEOUT if transmission timed out.
 */
HAL_StatusTypeDef can_msg_ack(uint32_t can_id, uint32_t timeout)
{
    CAN_TxHeaderTypeDef tx_header = {0};
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
/**
 * @brief CAN RX FIFO 0 message pending callback.
 *
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 */
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
