/**
 * @file    can_driver.h
 * @brief   CAN driver for BMS.
 * @details
 *
 * @author  CesarO
 * @date    2025-11-13
 *
 * @copyright Copyright (c) 2025 Ekteli.
 */

#ifndef INC_CAN_DRIVER_H_
#define INC_CAN_DRIVER_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32l4xx_hal.h"


/* Exported types ------------------------------------------------------------*/
typedef struct
{
    CAN_RxHeaderTypeDef header;
    uint8_t             data[8];
} can_message_t;


/* Exported constants and defines --------------------------------------------*/
#define CAN_ID_BMS (0x10)    // BMS CAN Node ID

/* Exported macros -----------------------------------------------------------*/


/* Exported function prototypes ----------------------------------------------*/
/**
 * @brief Transmit BMS state message.
 * @param soc State of Charge (%).
 * @param soh State of Health (%).
 * @param voltage Total pack voltage (mV).
 * @param current Pack current (mA).
 * @param temp_avg Average temperature (°C).
 */
void can_transmit_status(uint8_t soc, uint8_t soh, uint16_t voltage, int16_t current, int8_t temp_avg);

void can_decode_cmd(can_message_t *msg);

HAL_StatusTypeDef can_msg_transmit(uint32_t can_id, uint8_t *pdata, uint32_t length, uint32_t timeout);


#endif /* INC_CAN_DRIVER_H_ */
