/**
 * @file    bms.c
 * @brief
 * @details
 *
 * @author  CesarO
 * @date    2025-10-08
 */

/* Private includes ----------------------------------------------------------*/
#include <stdint.h>
#include "bms.h"


/* Private macros ------------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/


/* Exported types and variables ----------------------------------------------*/


/* Private (static) variables ------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/


/* Public user code ----------------------------------------------------------*/
/**
 * The device will automatically wake the internal oscillator at a falling edge
 * of SPI_CS, but it may take up to 50 µs to stabilize and be available for use
 * to the SPI interface logic.
 *
 * It is recommended to limit the frequency of SPI transactions by providing
 * 50 μs or more from the end of one transaction to the start of a new transaction.
 *
 * The first byte of a SPI transaction consists of an R/W bit (R = 0, W = 1),
 * followed by a 7-bit address, MSB first. If the controller (host) is writing,
 * then the second byte is the data written. If the controller is reading, then
 * the second byte sent on SPI_MOSI is ignored (except for CRC calculation).
 *
 * reg0 must be enabled and also set the voltages for reg1 and reg2
 **/
void bms_init(void)
{
}

/**
 * @brief Reset or shutdown BMS monitor
 * @details During normal operation, the RST_SHUT pin should be driven low.
 * When the pin is driven high, the BMS will immediately reset most of the
 * digital logic, including that associated with the serial communications bus.
 * However, it does not reset the logic that holds the state of the protection
 * FETs and FUSE, these remain as they were before the pin was driven high. If
 * the pin is driven high for 1 second, the device will transition into
 * SHUTDOWN mode, disabling external protection FETs, powering off the internal
 * oscillators, the REG18 LDO, the on-chip preregulator, and the REG1 and
 * REG2 LDOs, preregulator, and the REG1 and REG2 LDOs.
 */
void bms_reset_shutdown(void)
{
}

/**
 * @brief Disable discharge FETs
 * @details The DCHG pin is used to control the external discharge FETs.
 * assert the DFETOFF pin to keep the FETs off. As long as the pin is asserted,
 * the FETs are blocked from being reenabled. When the pin is deasserted,
 * the BQ76952 will reenable the FETs if nothing is blocking them being
 * reenabled (such as fault conditions still present, or the CFETOFF or
 * DFETOFF pins are asserted).
 * @note The DFETOFF or BOTHOFF functionality disables both the DSG FET and
 * the PDSG FET when asserted.
 */
void bms_dfet_off(void)
{
}

void bms_start_transaction(void)
{
    /* send CONTROL_STATUS() command to awaken bms */

    /* if bms respond not valid transaction, wait at least 135us and retry */

    /* read bms status and proceed normal program flow */
}

void bms_send_command(uint8_t address, uint8_t data)
{
    /* check wether write or read command and call the respective function */
    if (address & 0x80)
    {
        bms_write_register(address, data);
    }
    else
    {
        bms_read_register(address);
    }
}

void bms_write_register(uint8_t address, uint8_t data)
{
    uint8_t  tx_buffer[3];
    uint8_t  rx_buffer[3];
    uint16_t crc;

    tx_buffer[0] = SPI_WRITE_FRAME(address);
    tx_buffer[1] = data;

#if 0
    if(crc_enabled)
    {
        crc = crc8(tx_buffer, 2);
        tx_buffer[2] = crc;
    }

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // CS LOW
    HAL_Delay(1); // tCSS 50us min
    HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 3, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // CS HIGH
#endif

    // Check response in rx_buffer if needed
}

void bms_read_register(uint8_t address)
{
    uint8_t  tx_buffer[3];
    uint8_t  rx_buffer[3];
    uint16_t crc;

    tx_buffer[0] = SPI_READ_FRAME(address);
    tx_buffer[1] = SPI_DUMMY_BYTE;

#if 0
    if(crc_enabled)
    {
        crc = crc8(tx_buffer, 2);
        tx_buffer[2] = crc;
    }

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // CS LOW
    HAL_Delay(1); // tCSS 50us min
    HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 3, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // CS HIGH
#endif

    // Process received data in rx_buffer if needed
}

void BQ769x2_SetRegister(uint16_t reg_addr, uint32_t reg_data, uint8_t datalen)
{
    uint8_t TX_Buffer[2]  = {0x00, 0x00};
    uint8_t TX_RegData[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    // TX_RegData in little endian format
    TX_RegData[0] = reg_addr & 0xff;
    TX_RegData[1] = (reg_addr >> 8) & 0xff;
    TX_RegData[2] = reg_data & 0xff;    // 1st byte of data

    switch (datalen)
    {
        case 1:    // 1 byte datalength
            SPI_WriteReg(0x3E, TX_RegData, 3);
            delayUS(2000);
            TX_Buffer[0] = Checksum(TX_RegData, 3);
            TX_Buffer[1] = 0x05;                 // combined length of register address and data
            SPI_WriteReg(0x60, TX_Buffer, 2);    // Write the checksum and length
            delayUS(2000);
            break;
        case 2:    // 2 byte datalength
            TX_RegData[3] = (reg_data >> 8) & 0xff;
            SPI_WriteReg(0x3E, TX_RegData, 4);
            delayUS(2000);
            TX_Buffer[0] = Checksum(TX_RegData, 4);
            TX_Buffer[1] = 0x06;                 // combined length of register address and data
            SPI_WriteReg(0x60, TX_Buffer, 2);    // Write the checksum and length
            delayUS(2000);
            break;
        case 4:    // 4 byte datalength, Only used for CCGain and Capacity Gain
            TX_RegData[3] = (reg_data >> 8) & 0xff;
            TX_RegData[4] = (reg_data >> 16) & 0xff;
            TX_RegData[5] = (reg_data >> 24) & 0xff;
            SPI_WriteReg(0x3E, TX_RegData, 6);
            delayUS(2000);
            TX_Buffer[0] = Checksum(TX_RegData, 6);
            TX_Buffer[1] = 0x08;                 // combined length of register address and data
            SPI_WriteReg(0x60, TX_Buffer, 2);    // Write the checksum and length
            delayUS(2000);
            break;
    }
}

void CommandSubcommands(uint16_t command)    // For Command only Subcommands
// See the TRM or the BQ76952 header file for a full list of Command-only subcommands
{    // For DEEPSLEEP/SHUTDOWN subcommand you will need to call this function twice consecutively

    uint8_t TX_Reg[2] = {0x00, 0x00};

    // TX_Reg in little endian format
    TX_Reg[0] = command & 0xff;
    TX_Reg[1] = (command >> 8) & 0xff;

    SPI_WriteReg(0x3E, TX_Reg, 2);
    delayUS(2000);
}

void Subcommands(uint16_t command, uint16_t data, uint8_t type)
// See the TRM or the BQ76952 header file for a full list of Subcommands
{
    // security keys and Manu_data writes dont work with this function (reading these commands works)
    // max readback size is 32 bytes i.e. DASTATUS, CUV/COV snapshot
    uint8_t TX_Reg[4]    = {0x00, 0x00, 0x00, 0x00};
    uint8_t TX_Buffer[2] = {0x00, 0x00};

    // TX_Reg in little endian format
    TX_Reg[0] = command & 0xff;
    TX_Reg[1] = (command >> 8) & 0xff;

    if (type == R)
    {    // read
        SPI_WriteReg(0x3E, TX_Reg, 2);
        delayUS(2000);
        SPI_ReadReg(0x40, RX_32Byte, 32);    // RX_32Byte is a global variable
    }
    else if (type == W)
    {
        // FET_Control, REG12_Control
        TX_Reg[2] = data & 0xff;
        SPI_WriteReg(0x3E, TX_Reg, 3);
        delayUS(1000);
        TX_Buffer[0] = Checksum(TX_Reg, 3);
        TX_Buffer[1] = 0x05;    // combined length of registers address and data
        SPI_WriteReg(0x60, TX_Buffer, 2);
        delayUS(1000);
    }
    else if (type == W2)
    {    // write data with 2 bytes
        // CB_Active_Cells, CB_SET_LVL
        TX_Reg[2] = data & 0xff;
        TX_Reg[3] = (data >> 8) & 0xff;
        SPI_WriteReg(0x3E, TX_Reg, 4);
        delayUS(1000);
        TX_Buffer[0] = Checksum(TX_Reg, 4);
        TX_Buffer[1] = 0x06;    // combined length of registers address and data
        SPI_WriteReg(0x60, TX_Buffer, 2);
        delayUS(1000);
    }
}

void DirectCommands(uint8_t command, uint16_t data, uint8_t type)
// See the TRM or the BQ76952 header file for a full list of Direct Commands
{    // type: R = read, W = write
    uint8_t TX_data[2] = {0x00, 0x00};

    // little endian format
    TX_data[0] = data & 0xff;
    TX_data[1] = (data >> 8) & 0xff;

    if (type == R)
    {                                        // Read
        SPI_ReadReg(command, RX_data, 2);    // RX_data is a global variable
        delayUS(2000);
    }
    if (type == W)
    {    // write
        // Control_status, alarm_status, alarm_enable all 2 bytes long
        SPI_WriteReg(command, TX_data, 2);
        delayUS(2000);
    }
}

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

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(&hspi1, TX_Buffer, rxdata, 2, 1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

        while ((match == 0) & (retries > 0))
        {
            delayUS(500);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
            HAL_SPI_TransmitReceive(&hspi1, TX_Buffer, rxdata, 2, 1);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
            if ((rxdata[0] == addr) & (rxdata[1] == reg_data[i]))
                match = 1;
            retries--;
        }
        match = 0;
        addr += 1;
        delayUS(500);
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

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(&hspi1, TX_Buffer, rxdata, 2, 1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

        while ((match == 0) & (retries > 0))
        {
            delayUS(500);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
            HAL_SPI_TransmitReceive(&hspi1, TX_Buffer, rxdata, 2, 1);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
            if (rxdata[0] == addr)
            {
                match       = 1;
                reg_data[i] = rxdata[1];
            }
            retries--;
        }
        match = 0;
        addr += 1;
        delayUS(500);
    }
}


/* Private user code ---------------------------------------------------------*/
unsigned char Checksum(unsigned char *ptr, unsigned char len)
// Calculates the checksum when writing to a RAM register. The checksum is the inverse of the sum of the bytes.
{
    unsigned char i;
    unsigned char checksum = 0;

    for (i = 0; i < len; i++)
        checksum += ptr[i];

    checksum = 0xff & ~checksum;

    return (checksum);
}
