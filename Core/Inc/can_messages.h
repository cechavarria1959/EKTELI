/*
 * can_messages.h
 *
 *  Created on: Nov 13, 2025
 *      Author: CesarO
 */

/**
 * @file    can_messages.h
 * @brief   CAN message definitions for BMS.
 * @details
 *
 * @author  CesarO
 * @date    2025-11-13
 *
 * @copyright Copyright (c) 2025 Ekteli.
 */

#ifndef INC_CAN_MESSAGES_H_
#define INC_CAN_MESSAGES_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>


/* Exported types ------------------------------------------------------------*/
typedef enum
{
    BMS_STATE_INACTIVE    = 0,
    BMS_STATE_ACTIVE      = 1,
    BMS_STATE_SLEEP       = 2,
    BMS_STATE_FAULT       = 3,
    BMS_STATE_UNSPECIFIED = 4
} bms_state_t;

typedef enum
{
    BMS_CMD_NONE = 0
} bms_command_t;

/* Exported constants and defines --------------------------------------------*/
/* CAN IDs */
/* BMS Outgoing messages */
#define CAN_ID_BATTERY_STATUS 0x100u
#define CAN_ID_BMS_SAFETY     0x101u
#define CAN_ID_BMS_OPERATION  0x102u
#define CAN_ID_BMS_FW_VER     0x103u

/* BMS Incoming and Respond messages */
#define CAN_ID_BMS_STATE           0x150u
#define CAN_ID_BMS_BALANCE         0x151u
#define CAN_ID_BMS_SET_PROTECTIONS 0x152u
#define CAN_ID_BMS_GET_PROTECTIONS 0x153u
#define CAN_ID_BMS_PROTECTIONS     0x154u
#define CAN_ID_BMS_GET_FAULTS      0x155u
#define CAN_ID_BMS_FAULTS          0x156u
#define CAN_ID_BMS_GET_FW_VER      0x157u
#define CAN_ID_BMS_RESET           0x158u

/* OTA Mode */
#define CAN_ID_FW_UPDATE 0x159u

/* Bitfields */
#define PROTECTION_OV (1u << 0)
#define PROTECTION_UV (1u << 1)
#define PROTECTION_OT (1u << 2)
#define PROTECTION_OC (1u << 3)


/* Exported macros -----------------------------------------------------------*/


/* Exported function prototypes ----------------------------------------------*/


#endif /* INC_CAN_MESSAGES_H_ */
