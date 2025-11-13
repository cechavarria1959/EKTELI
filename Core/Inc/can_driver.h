/*
 * can_driver.h
 *
 *  Created on: Nov 13, 2025
 *      Author: CesarO
 */

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


/* Exported types ------------------------------------------------------------*/


/* Exported constants and defines --------------------------------------------*/


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


#endif /* INC_CAN_DRIVER_H_ */
