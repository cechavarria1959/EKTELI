/**
 * @file    bms.c
 * @brief   Battery Management System (BMS) driver
 * @details This file contains the implementation of the BMS driver functions,
 *          including initialization, configuration, and data retrieval from the
 *          BQ76952 battery monitor IC.
 *
 * @author  CesarO
 * @date    2025-10-08
 */

/* Private includes ----------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include "bms.h"
#include "main.h"
#include "spi.h"


/* Private macros ------------------------------------------------------------*/
#define R  0    // Read; Used in DirectCommands and Subcommands functions
#define W  1    // Write; Used in DirectCommands and Subcommands functions
#define W2 2    // Write data with two bytes; Used in Subcommands function

#define EMPTY_BUFFER ("\0\0\0\0")

#define SPI_BMS_REG12_CONFIG (0xDF)


/* Private typedef -----------------------------------------------------------*/


/* Exported types and variables ----------------------------------------------*/


/* Private (static) variables ------------------------------------------------*/
uint8_t rxdata[4] = {0};
uint8_t RX_32Byte[32] = {0};

uint16_t CellVoltage[16] = {0};

uint8_t value_PFStatusA;    // Permanent Fail Status Register A
uint8_t value_PFStatusB;    // Permanent Fail Status Register B
uint8_t value_PFStatusC;    // Permanent Fail Status Register C

uint8_t value_SafetyStatusA;    // Safety Status Register A
uint8_t value_SafetyStatusB;    // Safety Status Register B
uint8_t value_SafetyStatusC;    // Safety Status Register C

uint8_t FET_Status;    // FET Status register contents  - Shows states of FETs

uint8_t UV_Fault = 0;    // under-voltage fault state
uint8_t OV_Fault = 0;    // over-voltage fault state
uint8_t OT_Fault = 0;    // over-temperature fault state
uint8_t OC_Fault = 0;    // over-current fault state
uint8_t PF_Fault = 0;    // permanent-fault state

uint8_t ProtectionsTriggered    = 0;    // Set to 1 if any protection triggers
uint8_t PermanentFaultTriggered = 0;    // Set to 1 if any permanent fault triggers

uint8_t DSG = 0;    // discharge FET state
uint8_t CHG = 0;    // charge FET state

/* Private function prototypes -----------------------------------------------*/
unsigned char Checksum(unsigned char *ptr, unsigned char len);


/* Public user code ----------------------------------------------------------*/
/**
 * @brief Initializes the BMS (Battery Management System) configuration
 *        registers.
 *
 * This function configures the BQ769x2 battery monitor IC by setting up
 * various registers to define protection thresholds, pin configurations,
 * cell balancing, and other operational parameters. The configuration
 * includes enabling protections for over-current, over-voltage,
 * under-voltage, temperature limits, and setting up the cell count and
 * pin behaviors.
 *
 * Register configurations performed:
 * - Enters configuration update mode.
 * - Sets power configuration for best performance.
 * - Configures DFETOFF, ALERT, TS1, TS3, DCHG, and DDSG pins.
 * - Enables 10-cell operation (for 10S battery packs).
 * - Enables various protection features (short-circuit, over-current,
 *   over/under-voltage, temperature).
 * - Configures cell balancing in Relax and Charge modes.
 * - Sets thresholds for cell under-voltage (CUV), over-voltage (COV),
 *   over-current in charge/discharge (OCC/OCD1), and short-circuit (SCD).
 * - Sets delays and latch limits for short-circuit protection.
 * - Sets temperature thresholds for over-temperature in charge (OTC)
 *   and recovery.
 * - Optionally sets under-temperature in discharge (UTD) threshold
 *   (currently disabled).
 * - Exits configuration update mode.
 */
void bms_init()
{
    CommandSubcommands(ADDR_SET_CFGUPDATE);

    // Set DSLP_LDO & Set wake speed bits to 00 for best performance
    BQ769x2_SetRegister(POWER_CONFIG, 0x2D80, 2);

    // Set DFETOFF pin to BOTHOFF
    BQ769x2_SetRegister(DFETOFF_PIN_CONFIG, 0x42, 1);

    // ALERT pin drives high (REG1 voltage)when a protection has triggered
    BQ769x2_SetRegister(ALERT_PIN_CONFIG, 0x2A, 1);

    // Set TS1 to measure Cell Temperature
    BQ769x2_SetRegister(TS1_CONFIG, 0x07, 1);

    // Set TS3 to measure Cell Temperature
    BQ769x2_SetRegister(TS3_CONFIG, 0x07, 1);

    // Set DCHG pin Active Low
    BQ769x2_SetRegister(DCHG_PIN_CONFIG, 0x8B, 1);

    // Set DDSG pin Active Low
    BQ769x2_SetRegister(DDSG_PIN_CONFIG, 0x8B, 1);

    // Enable 10 cells
    // VC16-VC15, VC9-VC0. VC15-VC9 shorted for 10S pack
    BQ769x2_SetRegister(VCELL_MODE, 0x81FF, 2);

    // Enables SCD (short-circuit), OCD1 (over-current in discharge),
    // OCC (over-current in charge), COV (over-voltage), CUV (under-voltage)
    BQ769x2_SetRegister(ENABLED_PROTECTIONS_A, 0xBC, 1);

    // Enables OTINT (internal over-temperature), OTD (over-temperature in discharge),
    // OTC (over-temperature in charge), UTINT (internal under-temperature),
    // UTD (under-temperature in discharge), UTC (under-temperature in charge)
    BQ769x2_SetRegister(ENABLED_PROTECTIONS_B, 0x77, 1);

    // Balancing while in Relax or Charge modes
    BQ769x2_SetRegister(BALANCING_CONFIGURATION, 0x03, 1);

    // Set up CUV at 2479 mV
    BQ769x2_SetRegister(CUV_THRESHOLD, 0x31, 1);

    // Set up COV at 4200 mV
    BQ769x2_SetRegister(COV_THRESHOLD, 0x53, 1);

    // Set up OCC at 6A (6.72A) BMS current limit for 2P pack -> 0x03.
    // for 3P pack can reach 10A
    BQ769x2_SetRegister(OCC_THRESHOLD, 0x03, 1);

    // Set up OCD1 at 14A (14.4A) BMS current limit for 2P pack -> 0x03.
    // for 3P pack can reach 21.6A
    BQ769x2_SetRegister(OCD1_THRESHOLD, 0x07, 1);

    // Set up SCD at 100A
    BQ769x2_SetRegister(SCD_THRESHOLD, 0x05, 1);

    // Set up SCD Delay 30us
    BQ769x2_SetRegister(SCD_DELAY, 0x03, 1);

    // Set up SCDL Latch Limit to 1 to set SCD recovery only with load removal 0x9295 = 0x01
    // If this is not set, then SCD will recover based on time (SCD Recovery Time parameter).
    BQ769x2_SetRegister(SCDL_LATCH_LIMIT, 0x01, 1);

    // Set up OTC at 45degC
    BQ769x2_SetRegister(OTC_THRESHOLD, 0x2D, 1);

    // Set OTC recovery at 45degC
    BQ769x2_SetRegister(OTC_RECOVERY, 0x2D, 1);

    // Set UTD at -20degC
    // BQ769x2_SetRegister(UTD_THRESHOLD, 0xEC, 1); // Disabled for further data format correctness

    CommandSubcommands(ADDR_EXIT_CFGUPDATE);
}

/**
 * @brief Executes a direct command via SPI interface.
 *
 * This function sends a direct command with optional data to the device
 * using the SPI interface. The data is sent in little endian format.
 * Depending on the 'type' parameter, the function will either read from
 * or write to the device.
 *
 * @param command The command byte to send.
 * @param data    The 16-bit data to send.
 * @param type    Operation type: R for read, W for write.
 */
void DirectCommands(uint8_t command, uint16_t data, uint8_t type)
{
    uint8_t TX_data[2] = {0x00};

    // little endian format
    TX_data[0] = data & 0xff;
    TX_data[1] = (data >> 8) & 0xff;

    if (type == R)
    {
        SPI_ReadReg(command, RX_32Byte, 2, rxdata);
        HAL_Delay(2);
    }
    if (type == W)
    {
        SPI_WriteReg(command, TX_data, 2, rxdata);
        HAL_Delay(2);
    }
}

/**
 * @brief Sends a subcommand using the Command register.
 *
 * This function sends a subcommand to the device using the Command
 * register (0x3E).
 * 
 * Note: For certain subcommands like DEEPSLEEP or SHUTDOWN,
 * this function must be called twice consecutively.
 *
 * @param command The 16-bit subcommand to send.
 */
void CommandSubcommands(uint16_t command)
{
    uint8_t TX_Reg[2] = {0x00, 0x00};

    TX_Reg[0] = command & 0xff;
    TX_Reg[1] = (command >> 8) & 0xff;

    SPI_WriteReg(0x3E, TX_Reg, 2, rxdata);
    HAL_Delay(2);
}

void read_cuv_voltages()
{
    Subcommands(ADDR_CUV_SNAPSHOT, 0, 0);
    volatile uint16_t celdas[16] = {0};
    memcpy(celdas, RX_32Byte, 2);
    __NOP();
}

/**
 * @brief Executes a subcommand with optional data via SPI.
 *
 * This function sends a subcommand with optional data to the device
 * using the SPI interface. It supports read and write operations,
 * including writing 1 or 2 bytes of data. The function handles
 * checksum calculation and formatting of the data in little endian.
 * Note: Security keys and manufacturer data writes are not supported
 * with this function (reading is supported). Maximum readback size is
 * 32 bytes.
 *
 * @param command The 16-bit subcommand to send.
 * @param data    The 16-bit data to send.
 * @param type    Operation type: R for read, W for write (1 byte),
 *                W2 for write (2 bytes).
 */
void Subcommands(uint16_t command, uint16_t data, uint8_t type)
{
    uint8_t TX_Reg[4]    = {0};
    uint8_t TX_Buffer[2] = {0};

    TX_Reg[0] = command & 0xff;
    TX_Reg[1] = (command >> 8) & 0xff;

    if (type == R)
    {
        SPI_WriteReg(0x3E, TX_Reg, 2, rxdata);
        HAL_Delay(2);
        SPI_ReadReg(0x40, RX_32Byte, 32, rxdata);
    }
    else if (type == W)
    {
        TX_Reg[2] = data & 0xff;
        SPI_WriteReg(0x3E, TX_Reg, 3, rxdata);
        HAL_Delay(1);
        TX_Buffer[0] = Checksum(TX_Reg, 3);
        TX_Buffer[1] = 0x05;    // combined length of registers address and data
        SPI_WriteReg(0x60, TX_Buffer, 2, rxdata);
        HAL_Delay(1);
    }
    else if (type == W2)
    {
        TX_Reg[2] = data & 0xff;
        TX_Reg[3] = (data >> 8) & 0xff;
        SPI_WriteReg(0x3E, TX_Reg, 4, rxdata);
        HAL_Delay(1);
        TX_Buffer[0] = Checksum(TX_Reg, 4);
        TX_Buffer[1] = 0x06;
        SPI_WriteReg(0x60, TX_Buffer, 2, rxdata);
        HAL_Delay(1);
    }
}

/**
 * @brief Writes data to a BQ769x2 register via SPI interface.
 *
 * This function sets a register in the BQ769x2 battery management system
 * by sending the register address and data in little endian format. The
 * function supports writing 1, 2, or 4 bytes of data, and handles the
 * required checksum and command sequence for each case.
 *
 * @param reg_addr 16-bit register address to write to.
 * @param reg_data 32-bit data to write to the register.
 * @param datalen  Number of bytes to write (1, 2, or 4).
 *
 * @note For datalen = 4, only used for CCGain and Capacity Gain registers.
 */
void BQ769x2_SetRegister(uint16_t reg_addr, uint32_t reg_data, uint8_t datalen)
{
    uint8_t TX_Buffer[2]  = {0};
    uint8_t TX_RegData[6] = {0};

    TX_RegData[0] = reg_addr & 0xff;
    TX_RegData[1] = (reg_addr >> 8) & 0xff;
    TX_RegData[2] = reg_data & 0xff;

    switch (datalen)
    {
        case 1:
            SPI_WriteReg(0x3E, TX_RegData, 3, rxdata);
            HAL_Delay(2);
            TX_Buffer[0] = Checksum(TX_RegData, 3);
            TX_Buffer[1] = 0x05; 
            SPI_WriteReg(0x60, TX_Buffer, 2, rxdata);
            HAL_Delay(2);
            break;

        case 2:
            TX_RegData[3] = (reg_data >> 8) & 0xff;
            SPI_WriteReg(0x3E, TX_RegData, 4, rxdata);
            HAL_Delay(2);
            TX_Buffer[0] = Checksum(TX_RegData, 4);
            TX_Buffer[1] = 0x06; 
            SPI_WriteReg(0x60, TX_Buffer, 2, rxdata);
            HAL_Delay(2);
            break;

        case 4:
            TX_RegData[3] = (reg_data >> 8) & 0xff;
            TX_RegData[4] = (reg_data >> 16) & 0xff;
            TX_RegData[5] = (reg_data >> 24) & 0xff;
            SPI_WriteReg(0x3E, TX_RegData, 6, rxdata);
            HAL_Delay(2);
            TX_Buffer[0] = Checksum(TX_RegData, 6);
            TX_Buffer[1] = 0x08; 
            SPI_WriteReg(0x60, TX_Buffer, 2, rxdata);
            HAL_Delay(2);
            break;

        default:
            /* nothing to do */
            break;
    }
}

/**
 * @brief Reads voltage from BQ769x2 device for a given command address.
 *
 * @param command Command address to read voltage from.
 * @return uint16_t Voltage value in mV or centivolts, depending on address.
 */
uint16_t BQ769x2_ReadVoltage(uint8_t command)
{
    DirectCommands(command, 0x00, R);

    // Cells 1 through 16 (0x14 to 0x32)
    if (command >= ADDR_CELL_VOLTAGES && command <= (ADDR_CELL_VOLTAGES + (15 * 2)))
    {
        return (RX_32Byte[1] * 256 + RX_32Byte[0]);    // voltage is reported in mV
    }
    else
    {
        // voltage is reported in 0.01V (centivolts) units by default,
        //(Settings:Configuration:DA Configuration USER_VOLTS_CV in TRM)
        return 10 * (RX_32Byte[1] * 256 + RX_32Byte[0]);
    }
}

/**
 * @brief Reads cell voltages from BQ769x2 based on BMS model configuration.
 *
 * For BMS_MODEL_10S: Reads cells 1-9 consecutively, cell 10 from address 16.
 * For BMS_MODEL_14S: Reads cells 1-13 consecutively, cell 14 from address 16.
 * The last cell is always read from the Cell 16 address due to BQ769x2 wiring.
 */
void BQ769x2_Readcell_voltages(void)
{
    uint8_t cell_addr = ADDR_CELL_VOLTAGES;    /* Cell1Voltage is 0x14 */

    /* Read consecutive cells (1 to N-1) */
    for (uint8_t cell = 0; cell < BMS_CONSECUTIVE_CELLS; cell++)
    {
        CellVoltage[cell] = BQ769x2_ReadVoltage(cell_addr);
        cell_addr += 2;
    }

    /* Read last cell from Cell 16 address */
    cell_addr = ADDR_CELL_VOLTAGES + (BMS_LAST_CELL_INDEX * 2);
    CellVoltage[BMS_CELL_COUNT - 1] = BQ769x2_ReadVoltage(cell_addr);
}

void BQ769x2_readall_voltages(void)
{
    int cellvoltageholder = ADDR_CELL_VOLTAGES;    // Cell1Voltage is 0x14

    volatile uint16_t allvoltages[16];

    for (int x = 0; x < 16; x++)
    {
        allvoltages[x] = BQ769x2_ReadVoltage(cellvoltageholder);
        cellvoltageholder += 2;
    }

    UNUSED(allvoltages[0]);

    __NOP();
}

/**
 * @brief Reads current value from BQ769x2 device.
 *
 * Reads the current measurement from the device and returns it in milliamps
 * (mA).
 *
 * @return int16_t Current value in mA.
 */
int16_t BQ769x2_ReadCurrent()
{
    DirectCommands(ADDR_CC2_CURRENT, 0x00, R);
    return (RX_32Byte[1] * 256 + RX_32Byte[0]);    // current is reported in mA
}

/**
 * @brief Reads temperature from BQ769x2 device for a given command address.
 *
 * Converts the raw temperature value from 0.1 Kelvin units to Celsius.
 *
 * @param command Command address to read temperature from.
 * @return float Temperature in degrees Celsius.
 */
float BQ769x2_ReadTemperature(uint8_t command)
{
    DirectCommands(command, 0x00, R);

    return (0.1 * (float)(RX_32Byte[1] * 256 + RX_32Byte[0])) - 273.15;    // converts from 0.1K to Celcius
}

/**
 * @brief Reads the Safety Status registers (A/B/C) from the BQ769x2 device.
 *
 * This function checks which primary protections have been triggered by reading
 * the Safety Status A and B registers. It extracts Overvoltage (OV), 
 * Undervoltage (UV), and Overcurrent (OC) fault bits. Overtemperature (OT) 
 * faults are checked from Safety Status B. If any protection is triggered, 
 * the ProtectionsTriggered flag is set.
 */
void BQ769x2_ReadSafetyStatus()
{
    DirectCommands(ADDR_SAFETY_STATUS_A, 0x00, R);
    value_SafetyStatusA = RX_32Byte[0];

    UV_Fault = (0x4 & RX_32Byte[0]) >> 2;
    OV_Fault = (0x8 & RX_32Byte[0]) >> 3;
    // OCC_Fault = (0x10 & RX_32Byte[0]) >> 4;
    // OCD1_Fault = (0x20 & RX_32Byte[0]) >> 5;
    // OCD2_Fault = (0x40 & RX_32Byte[0]) >> 6;
    // SCD_Fault = (0x80 & RX_32Byte[0]) >> 7;

    // check if any over-current bits are set
    if ((RX_32Byte[0] & 0xF0) != 0)
    {
        OC_Fault = 1;
    }
    else
    {
        OC_Fault = 0;
    }

    DirectCommands(ADDR_SAFETY_STATUS_B, 0x00, R);
    value_SafetyStatusB = RX_32Byte[0];

    // check if any over-temperature bits are set
    if ((RX_32Byte[0] & 0xF0) != 0)
    {
        OT_Fault = 1;
    }
    else
    {
        OT_Fault = 0;
    }

    /* Safety Status C needed? */
     DirectCommands(ADDR_SAFETY_STATUS_C, 0x00, R);
     value_SafetyStatusC = RX_32Byte[0];

    if ((value_SafetyStatusA + value_SafetyStatusB + value_SafetyStatusC) > 0)
    {
        ProtectionsTriggered = 1;
    }
    else
    {
        ProtectionsTriggered = 0;
    }

    /* Read safety alert statuses */
    DirectCommands(0x02, 0, R); //safety alert A
    DirectCommands(0x04, 0, R); //safety alert B
    DirectCommands(0x06, 0, R); //safety alert C
}

/**
 * @brief Reads the Permanent Fail Status registers (A/B/C) from the BQ769x2.
 *
 * This function checks which permanent failures have been triggered by reading
 * the Permanent Fail Status A, B, and C registers. If any permanent fault is 
 * detected, the PermanentFaultTriggered flag is set.
 */
void BQ769x2_ReadPFStatus()
{
    DirectCommands(ADDR_PF_STATUS_A, 0x00, R);
    value_PFStatusA = RX_32Byte[0];

    DirectCommands(ADDR_PF_STATUS_B, 0x00, R);
    value_PFStatusB = RX_32Byte[0];
    
    DirectCommands(ADDR_PF_STATUS_C, 0x00, R);
    value_PFStatusC = RX_32Byte[0];

    if ((value_PFStatusA + value_PFStatusB + value_PFStatusC) > 0)
    {
        PermanentFaultTriggered = 1;
    }
    else
    {
        PermanentFaultTriggered = 0;
    }

    DirectCommands(0x11, 0, R); // pf status D

    DirectCommands(0x0A, 0, R); // pf alert A
    DirectCommands(0x0C, 0, R); // pf alert B
    DirectCommands(0x0E, 0, R); // pf alert C
    DirectCommands(0x10, 0, R); // pf alert D
}

/**
 * @brief Reads the Alarm Status register from the BQ769x2 device.
 *
 * @return uint16_t Value of the Alarm Status register.
 */
uint16_t BQ769x2_ReadAlarmStatus()
{
    DirectCommands(ADDR_ALARM_STATUS, 0x00, R);
    return (RX_32Byte[1] * 256 + RX_32Byte[0]);
}

/**
 * @brief Reads the FET Status register from the BQ769x2 device.
 *
 * This function checks which FETs (DSG and CHG) are enabled by reading the FET
 * Status register and updates the corresponding status flags.
 */
void BQ769x2_ReadFETStatus()
{
    DirectCommands(ADDR_FET_STATUS, 0x00, R);
    FET_Status = RX_32Byte[0];

    CHG = (0x1 & RX_32Byte[0]);
    DSG = (0x4 & RX_32Byte[0]) >> 2;
}

/**
 * @brief Checks REG2 status and OTP programming conditions for BQ7695203.
 *
 * mainly checks REG2 Status for MCU power rail, which is off for
 * BQ7695203 device as default.
 *
 * In order for OTP to work a voltage between 10 to 12 V should be applied
 * to the BAT pin and the device must be in FULLACCESS mode. More details
 * can be found in BQ769x2 Calibration and OTP Programming Guide SLUAA32A.
 */
bms_otp_status_t bms_otp_check(void)
{
    bms_otp_status_t otp_status = BMS_OTP_ERROR;

    Subcommands(REG12_CONFIG, 0, R);
    if (RX_32Byte[0] != SPI_BMS_REG12_CONFIG)
    {
        if (memcmp(RX_32Byte, EMPTY_BUFFER, 4) == 0 &&
            memcmp(rxdata, EMPTY_BUFFER, 4) == 0)
        {
            return BMS_OTP_BQ_NOT_DETECTED;
        }

        DirectCommands(ADDR_BATTERY_STATUS, 0, R);
        if ((RX_32Byte[1] & 0x03) != 0x01)
        {
            /* Enter FULLACCESS mode sequence */
            /* By default device is in FULLACCESS */
            return BMS_OTP_ERROR;
        }

        CommandSubcommands(ADDR_SET_CFGUPDATE);
        BQ769x2_SetRegister(REG12_CONFIG, SPI_BMS_REG12_CONFIG, 1);    // REG2 @ 3.3V & REG1 @ 5V
        CommandSubcommands(ADDR_EXIT_CFGUPDATE);

        Subcommands(REG12_CONFIG, 0, R);
        if (RX_32Byte[0] != SPI_BMS_REG12_CONFIG)
        {
            return BMS_OTP_NOT_PROGRAMMED;
        }

        CommandSubcommands(ADDR_SET_CFGUPDATE);
        DirectCommands(ADDR_BATTERY_STATUS, 0, R);
        if ((RX_32Byte[0] & 0x80) >> 7)
        {
            // Conditions for OTP programming NOT met
            Subcommands(ADDR_OTP_WR_CHECK, 0x0000, R);    // check failed condition
            if (RX_32Byte[0] & 0x01)
            {
                // stack voltage is above the allowed OTP programming voltage
                HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 3722);    // ~3V
            }
            else if (RX_32Byte[0] & 0x02)
            {
                // stack voltage is below the allowed OTP programming voltage
                HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2480);    // ~2V
            }
            else if (RX_32Byte[0] & 0x04)
            {
                // internal temperature is above the allowed OTP programming temperature range
                HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 1240);    // ~1V
            }
            else
            {
                HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
            }

            CommandSubcommands(ADDR_EXIT_CFGUPDATE);
            return BMS_OTP_CONDITIONS_NOT_MET;
        }

        Subcommands(ADDR_OTP_WR_CHECK, 0x0000, R);

        uint8_t txreg[2];
        txreg[0] = ADDR_OTP_WR_CHECK & 0xff;
        txreg[1] = (ADDR_OTP_WR_CHECK >> 8) & 0xff;
        SPI_WriteReg(0x3E, txreg, 2, rxdata);
        HAL_Delay(100);
        SPI_ReadReg(0x40, RX_32Byte, 32, rxdata);
        if (RX_32Byte[0] != 0x80)
        {
            // OTP programming NOT possible
            CommandSubcommands(ADDR_EXIT_CFGUPDATE);
            return BMS_OTP_NOT_PROGRAMMED;
        }

        txreg[0] = ADDR_OTP_WRITE & 0xff;
        txreg[1] = (ADDR_OTP_WRITE >> 8) & 0xff;
        SPI_WriteReg(0x3E, txreg, 2, rxdata);
        HAL_Delay(200);
        SPI_ReadReg(0x40, RX_32Byte, 32, rxdata);

        if (RX_32Byte[0] != 0x80)
        {
            // OTP programming FAILED
            return BMS_OTP_FAILED;
        }
        CommandSubcommands(ADDR_EXIT_CFGUPDATE);

        otp_status = BMS_OTP_OK;
    }
    else
    {
        // OTP already programmed
        otp_status = BMS_OTP_OK;
    }

    return otp_status;
}

/**
 * @brief Returns the smallest cell voltage from the CellVoltage array.
 *
 * Iterates through all configured cells and finds the minimum value.
 *
 * @return Smallest cell voltage (uint16_t).
 */
uint16_t get_smallest_cell_voltage(void)
{
    uint16_t smallest_voltage = 0xFFFF;

    for (uint8_t cell = 0; cell < BMS_CELL_COUNT; cell++)
    {
        if (CellVoltage[cell] < smallest_voltage)
        {
            smallest_voltage = CellVoltage[cell];
        }
    }

    return smallest_voltage;
}

/**
 * @brief Returns the largest cell voltage from the CellVoltage array.
 *
 * Iterates through all configured cells and finds the maximum value.
 *
 * @return Largest cell voltage (uint16_t).
 */
uint16_t get_largest_cell_voltage(void)
{
    uint16_t largest_voltage = 0x0000;

    for (uint8_t cell = 0; cell < BMS_CELL_COUNT; cell++)
    {
        if (CellVoltage[cell] > largest_voltage)
        {
            largest_voltage = CellVoltage[cell];
        }
    }

    return largest_voltage;
}

/**
 * @brief Gets the BMS status.
 * 
 * Returns 3 if any protection or permanent fault is triggered, otherwise
 * returns 1 for normal operation.
 * 
 * @return BMS status (uint8_t): 3 = Fault, 1 = Normal.
 */
uint8_t get_bms_status(void)
{
    uint8_t rv = 0;

    if (ProtectionsTriggered || PermanentFaultTriggered)
    {
        rv = 3u;    // BMS in Fault State
    }
    else
    {
        rv = 1u;    // BMS Operating Normally
    }

    return rv;
}

/**
 * @brief Gets the cell balancing status.
 * 
 * Checks if any cell balancing is active by reading balancing active cells.
 * 
 * @return Balancing status (uint8_t): 1 = Active, 0 = Inactive.
 */
uint8_t get_balancing_status(void)
{
    uint8_t balancing_status = 0u;

    Subcommands(ADDR_CB_ACTIVE_CELLS, 0, R);

    uint16_t balancing_active_cells = (RX_32Byte[1] * 256 + RX_32Byte[0]);
    if (balancing_active_cells != 0x0000)
    {
        balancing_status = 1;
    }
    else
    {
        balancing_status = 0;
    }

    return balancing_status;
}

/**
 * @brief Gets the charging status.
 * 
 * Determines charging status based on FETs and current or voltage readings.
 * 
 * @return Charging status (uint8_t): 1 = Charging, 0 = Not charging.
 */
uint8_t get_charging_status(void)
{
    uint8_t charging_status = 0u;

    /* With the FETs on, the only way to detect charger is with the current
     * measurement. With FETs off you can compare the PACK voltage to top of
     * stack (cmds 0x36 and 0x34).
     **/

    uint8_t fets_status = DSG | CHG;
    if (fets_status == 0)
    {
        /* FETs off */
        uint16_t stack_voltage = BQ769x2_ReadVoltage(ADDR_STACK_VOLTAGE);
        uint16_t pack_voltage  = BQ769x2_ReadVoltage(ADDR_PACK_PIN_VOLTAGE);

        if (stack_voltage < pack_voltage)
        {
            charging_status = 1;
        }
    }
    else
    {
        /* FETs on */
        int16_t current = BQ769x2_ReadCurrent();
        if (current > 0)
        {
            charging_status = 1;
        }
    }

    return charging_status;
}

/**
 * @brief Gets BMS faults.
 * 
 * Returns a bitfield representing various fault conditions if any protection
 * or permanent fault is triggered.
 * 
 * @return Fault bitfield (uint8_t).
 */
uint8_t bms_get_faults(void)
{
    uint8_t faults = 0;

    if (ProtectionsTriggered || PermanentFaultTriggered)
    {
        faults = (PF_Fault << 4) | (OC_Fault << 3) | (OT_Fault << 2) | (UV_Fault << 1) | OV_Fault;
    }

    return faults;
}

/**
 * @brief Gets the FET status.
 * 
 * Returns the status of CHG and DSG FETs as a bitfield.
 * 
 * @return FET status (uint8_t).
 */
uint8_t get_fet_status(void)
{
    return (CHG << 1) | DSG;
}

/**
 * @brief Disable discharge FETs
 * 
 * @details The DCHG pin is used to control the external discharge FETs.
 * assert the DFETOFF pin to keep the FETs off. As long as the pin is asserted,
 * the FETs are blocked from being reenabled. When the pin is deasserted,
 * the BQ76952 will reenable the FETs if nothing is blocking them being
 * reenabled (such as fault conditions still present, or the CFETOFF or
 * DFETOFF pins are asserted).
 * 
 * @note The DFETOFF or BOTHOFF functionality disables both the DSG FET and
 * the PDSG FET when asserted.
 */
void bms_dfet_off(void)
{
}

/**
 * @brief Reset or shutdown BMS monitor
 * 
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


/* Private user code ---------------------------------------------------------*/
/**
 * @brief Calculates the checksum for a given byte array.
 *
 * The checksum is computed as the bitwise inverse of the sum of all bytes
 * in the input array.
 *
 * @param ptr Pointer to the byte array to be checked.
 * @param len Number of bytes in the array.
 * @return The calculated checksum as an unsigned char.
 */
unsigned char Checksum(unsigned char *ptr, unsigned char len)
// The checksum is the inverse of the sum of the bytes.
{
    unsigned char i;
    unsigned char checksum = 0;

    for (i = 0; i < len; i++)
    {
        checksum += ptr[i];
    }

    checksum = 0xff & ~checksum;

    return (checksum);
}
