/**
 * @file    spi.c
 * @brief
 * @details
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


/* Private typedef -----------------------------------------------------------*/


/* Exported types and variables ----------------------------------------------*/
extern uint8_t rxdata[4];


/* Private (static) variables ------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
unsigned char CRC8(unsigned char *ptr, unsigned char len);


/* Public user code ----------------------------------------------------------*/
void SPI_WriteReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
    // SPI Write. Includes retries in case HFO has not started or if wait time is needed. See BQ76952 Software Development Guide for examples
    uint8_t      addr;
    uint8_t      TX_Buffer[MAX_BUFFER_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    unsigned int i;
    unsigned int match;
    unsigned int retries = 10;

    match = 0;
    addr  = 0x80 | reg_addr;

    for (i = 0; i < count; i++)
    {
        TX_Buffer[0] = addr;
        TX_Buffer[1] = reg_data[i];
        TX_Buffer[2] = CRC8(TX_Buffer, 2);

        HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(&hspi1, TX_Buffer, rxdata, 3, 1);
        HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

        while ((match == 0) & (retries > 0))
        {
            HAL_Delay(1);
            HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
            HAL_SPI_TransmitReceive(&hspi1, TX_Buffer, rxdata, 3, 1);
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

void SPI_ReadReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
    // SPI Read. Includes retries in case HFO has not started or if wait time is needed. See BQ76952 Software Development Guide for examples
    uint8_t      addr;
    uint8_t      TX_Buffer[MAX_BUFFER_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    unsigned int i;
    unsigned int match;
    unsigned int retries = 10;

    match = 0;
    addr  = reg_addr;

    for (i = 0; i < count; i++)
    {
        TX_Buffer[0] = addr;
        TX_Buffer[1] = 0xFF;
        TX_Buffer[2] = CRC8(TX_Buffer, 2);

        HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(&hspi1, TX_Buffer, rxdata, 3, 1);
        HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

        while ((match == 0) & (retries > 0))
        {
            HAL_Delay(1);
            HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
            HAL_SPI_TransmitReceive(&hspi1, TX_Buffer, rxdata, 3, 1);
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
unsigned char CRC8(unsigned char *ptr, unsigned char len)
// Calculates CRC8 for passed bytes. Used in i2c read and write functions
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
    return (crc);
}
