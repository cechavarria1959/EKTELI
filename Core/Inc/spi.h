/**
 * @file    spi.h
 * @brief   SPI interface header for register read/write operations.
 * @details Provides function prototypes for reading and writing registers
 *          via SPI. Supports multi-byte operations and returns received
 *          data through provided buffers.
 *
 * @author  CesarO
 * @date    2026-02-07
 *
 * @copyright Copyright (c) 2025 Ekteli.
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>


/* Exported types ------------------------------------------------------------*/


/* Exported constants and defines --------------------------------------------*/


/* Exported macros -----------------------------------------------------------*/


/* Exported function prototypes ----------------------------------------------*/
void SPI_ReadReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count, uint8_t rxdata[4]);
void SPI_WriteReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count, uint8_t rxdata[4]);

#endif /* INC_SPI_H_ */
