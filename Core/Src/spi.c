/**
 * @file    spi.c
 * @brief   SPI communication routines for register access with CRC8 checking.
 * @details Provides functions to read and write registers over SPI, including
 *          retry mechanisms and CRC8 validation for robust communication.
 *          Implements CRC8 calculation for data integrity. Designed for use
 *          with STM32 HAL and custom BMS firmware.
 *
 * @author  CesarO
 * @date    2026-02-07
 *
 * @copyright Copyright (c) 2025 Ekteli.
 */

/* Private includes ----------------------------------------------------------*/
#include <stdint.h>
#include "main.h"
#include "spi.h"


/* Private macros ------------------------------------------------------------*/
#define MAX_BUFFER_SIZE (10)

#define SPI_READ_FRAME(addr)  (addr & 0x7F)
#define SPI_WRITE_FRAME(addr) ((addr & 0x7F) | 0x80)
#define SPI_DUMMY_BYTE        (0x00)
#define SPI_INVALID_RX_BUFFER (0xFFFF)

#define SPI_BMS_ERROR_16BIT   (0xFFFF)
#define SPI_CLOCK_NOT_POWERED (0xFFFFFF)
#define SPI_CRC_ERROR         (0xFFFFAA)
#define SPI_BMS_TIMEOUT       (0xFFFF00)


/* Private typedef -----------------------------------------------------------*/


/* Exported types and variables ----------------------------------------------*/


/* Private (static) variables ------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
unsigned char crc8(unsigned char *ptr, unsigned char len);


/* Public user code ----------------------------------------------------------*/
/**
 * @brief Writes data to a register over SPI with retries and CRC8 checking.
 *
 * @param reg_addr  The starting register address to write to.
 * @param reg_data  Pointer to the data buffer containing bytes to write.
 * @param count     Number of bytes to write.
 * @param rxdata    Pointer to a buffer to store received SPI data (4 bytes).
 */
void spi_write_register(uint8_t reg_addr, uint8_t *reg_data, uint8_t count, uint8_t rxdata[4])
{
    uint8_t      addr;
    uint8_t      tx_buffer[MAX_BUFFER_SIZE] = {0x00};
    unsigned int i;
    unsigned int match;
    unsigned int retries = 10;

    match = 0;
    addr  = 0x80 | reg_addr;

    for (i = 0; i < count; i++)
    {
        tx_buffer[0] = addr;
        tx_buffer[1] = reg_data[i];
        tx_buffer[2] = crc8(tx_buffer, 2);

        HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rxdata, 3, 1);
        HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

        while ((match == 0) & (retries > 0))
        {
            HAL_Delay(1);
            HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
            HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rxdata, 3, 1);
            HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
            if ((rxdata[0] == addr) & (rxdata[1] == reg_data[i]))
                match = 1;
            retries--;
        }
        match = 0;
        addr += 1;
        retries = 10;
        HAL_Delay(1);
    }
}

/**
 * @brief Reads data from a register over SPI with retries and CRC8 checking.
 *
 * @param reg_addr  The starting register address to read from.
 * @param reg_data  Pointer to the buffer where read data will be stored.
 * @param count     Number of bytes to read.
 * @param rxdata    Pointer to a buffer to store intermediate received SPI data (=4 bytes).
 *
 * @note for examples on transactions, can read SLUAA11B
 */
void spi_read_register(uint8_t reg_addr, uint8_t *reg_data, uint8_t count, uint8_t rxdata[4])
{
    uint8_t      addr;
    uint8_t      tx_buffer[MAX_BUFFER_SIZE] = {0x00};
    unsigned int i;
    unsigned int match;
    unsigned int retries = 10;

    match = 0;
    addr  = reg_addr;

    for (i = 0; i < count; i++)
    {
        tx_buffer[0] = addr;
        tx_buffer[1] = 0xFF;
        tx_buffer[2] = crc8(tx_buffer, 2);

        HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rxdata, 3, 1);
        HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

        while ((match == 0) & (retries > 0))
        {
            HAL_Delay(1);
            HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
            HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rxdata, 3, 1);
            HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
            if (rxdata[0] == addr)
            {
                match       = 1;
                reg_data[i] = rxdata[1];
            }
            retries--;
        }
        match = 0;
        addr += 1;
        HAL_Delay(1);
    }
}

/* Private user code ---------------------------------------------------------*/
/**
 * @brief Calculates CRC8 for a given data buffer.
 *
 * @param ptr   Pointer to the data buffer.
 * @param len   Length of the data buffer in bytes.
 * @return      The calculated CRC8 value.
 */
unsigned char crc8(unsigned char *ptr, unsigned char len)
{
    unsigned char i;
    unsigned char crc = 0;
    while (len-- != 0)
    {
        for (i = 0x80; i != 0; i /= 2)
        {
            if ((crc & 0x80) != 0)
            {
                crc *= 2;
                crc ^= 0x107;
            }
            else
                crc *= 2;

            if ((*ptr & i) != 0)
                crc ^= 0x107;
        }
        ptr++;
    }
    return crc;
}
