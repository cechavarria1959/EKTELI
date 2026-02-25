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
#define R  0    // Read; Used in direct_commands and subcommands functions
#define W  1    // Write; Used in direct_commands and subcommands functions
#define W2 2    // Write data with two bytes; Used in subcommands function

#define EMPTY_BUFFER ("\0\0\0\0")

#define SPI_BMS_REG12_CONFIG (0xDF)


/* Private typedef -----------------------------------------------------------*/


/* Exported types and variables ----------------------------------------------*/


/* Private (static) variables ------------------------------------------------*/
uint8_t rxdata[4]     = {0};
uint8_t rx_32byte[32] = {0};

uint16_t cell_voltage[16] = {0};

uint8_t value_pf_status_a;    // Permanent Fail Status Register A
uint8_t value_pf_status_b;    // Permanent Fail Status Register B
uint8_t value_pf_status_c;    // Permanent Fail Status Register C

uint8_t value_safety_status_a;    // Safety Status Register A
uint8_t value_safety_status_b;    // Safety Status Register B
uint8_t value_safety_status_c;    // Safety Status Register C

uint8_t FET_status;      // FET Status register contents  - Shows states of FETs
uint8_t UV_fault = 0;    // under-voltage fault state
uint8_t OV_fault = 0;    // over-voltage fault state
uint8_t OT_fault = 0;    // over-temperature fault state
uint8_t OC_fault = 0;    // over-current fault state
uint8_t PF_fault = 0;    // permanent-fault state

uint8_t protections_triggered     = 0;    // Set to 1 if any protection triggers
uint8_t permanent_fault_triggered = 0;    // Set to 1 if any permanent fault triggers

uint8_t DSG = 0;    // discharge FET state
uint8_t CHG = 0;    // charge FET state

/* Private function prototypes -----------------------------------------------*/
unsigned char checksum(unsigned char *ptr, unsigned char len);


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
    command_subcommands(ADDR_SET_CFGUPDATE);

    // Set DSLP_LDO & Set wake speed bits to 00 for best performance
    bq769x2_set_register(POWER_CONFIG, 0x2D80, 2);

    // Set DFETOFF pin to BOTHOFF
    bq769x2_set_register(DFETOFF_PIN_CONFIG, 0x42, 1);

    // ALERT pin drives high (REG1 voltage)when a protection has triggered
    bq769x2_set_register(ALERT_PIN_CONFIG, 0x2A, 1);

    // Set TS1 to measure Cell Temperature
    bq769x2_set_register(TS1_CONFIG, 0x07, 1);

    // Set TS3 to measure Cell Temperature
    bq769x2_set_register(TS3_CONFIG, 0x07, 1);

    // Set DCHG pin Active Low
    bq769x2_set_register(DCHG_PIN_CONFIG, 0x8B, 1);

    // Set DDSG pin Active Low
    bq769x2_set_register(DDSG_PIN_CONFIG, 0x8B, 1);

    // Enable 10 cells
    // VC16-VC15, VC9-VC0. VC15-VC9 shorted for 10S pack
    bq769x2_set_register(VCELL_MODE, 0x81FF, 2);

    // Enables SCD (short-circuit), OCD1 (over-current in discharge),
    // OCC (over-current in charge), COV (over-voltage), CUV (under-voltage)
    bq769x2_set_register(ENABLED_PROTECTIONS_A, 0xBC, 1);

    // Enables OTINT (internal over-temperature), OTD (over-temperature in discharge),
    // OTC (over-temperature in charge), UTINT (internal under-temperature),
    // UTD (under-temperature in discharge), UTC (under-temperature in charge)
    bq769x2_set_register(ENABLED_PROTECTIONS_B, 0x77, 1);

    // Balancing while in Relax or Charge modes
    bq769x2_set_register(BALANCING_CONFIGURATION, 0x03, 1);

    // Set up CUV at 2479 mV
    bq769x2_set_register(CUV_THRESHOLD, 0x31, 1);

    // Set up COV at 4200 mV
    bq769x2_set_register(COV_THRESHOLD, 0x53, 1);

    // Set up OCC at 6A (6.72A) BMS current limit for 2P pack -> 0x03.
    // for 3P pack can reach 10A
    bq769x2_set_register(OCC_THRESHOLD, 0x03, 1);

    // Set up OCD1 at 14A (14.4A) BMS current limit for 2P pack -> 0x03.
    // for 3P pack can reach 21.6A
    bq769x2_set_register(OCD1_THRESHOLD, 0x07, 1);

    // Set up SCD at 100A
    bq769x2_set_register(SCD_THRESHOLD, 0x05, 1);

    // Set up SCD Delay 30us
    bq769x2_set_register(SCD_DELAY, 0x03, 1);

    // Set up SCDL Latch Limit to 1 to set SCD recovery only with load removal 0x9295 = 0x01
    // If this is not set, then SCD will recover based on time (SCD Recovery Time parameter).
    bq769x2_set_register(SCDL_LATCH_LIMIT, 0x01, 1);

    // Set up OTC at 45degC
    bq769x2_set_register(OTC_THRESHOLD, 0x2D, 1);

    // Set OTC recovery at 45degC
    bq769x2_set_register(OTC_RECOVERY, 0x2D, 1);

    // Set UTD at -20degC
    // bq769x2_set_register(UTD_THRESHOLD, 0xEC, 1); // Disabled for further data format correctness

    command_subcommands(ADDR_EXIT_CFGUPDATE);
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
void direct_commands(uint8_t command, uint16_t data, uint8_t type)
{
    uint8_t txdata[2] = {0x00};

    // little endian format
    txdata[0] = data & 0xff;
    txdata[1] = (data >> 8) & 0xff;

    if (type == R)
    {
        spi_read_register(command, rx_32byte, 2, rxdata);
        HAL_Delay(2);
    }
    if (type == W)
    {
        spi_write_register(command, txdata, 2, rxdata);
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
void command_subcommands(uint16_t command)
{
    uint8_t tx_reg[2] = {0x00, 0x00};

    tx_reg[0] = command & 0xff;
    tx_reg[1] = (command >> 8) & 0xff;

    spi_write_register(0x3E, tx_reg, 2, rxdata);
    HAL_Delay(2);
}

void read_cuv_voltages()
{
    subcommands(ADDR_CUV_SNAPSHOT, 0, 0);
    volatile uint16_t celdas[16] = {0};
    memcpy(celdas, rx_32byte, 2);
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
void subcommands(uint16_t command, uint16_t data, uint8_t type)
{
    uint8_t tx_reg[4]    = {0};
    uint8_t tx_buffer[2] = {0};

    tx_reg[0] = command & 0xff;
    tx_reg[1] = (command >> 8) & 0xff;

    if (type == R)
    {
        spi_write_register(0x3E, tx_reg, 2, rxdata);
        HAL_Delay(2);
        spi_read_register(0x40, rx_32byte, 32, rxdata);
    }
    else if (type == W)
    {
        tx_reg[2] = data & 0xff;
        spi_write_register(0x3E, tx_reg, 3, rxdata);
        HAL_Delay(1);
        tx_buffer[0] = checksum(tx_reg, 3);
        tx_buffer[1] = 0x05;    // combined length of registers address and data
        spi_write_register(0x60, tx_buffer, 2, rxdata);
        HAL_Delay(1);
    }
    else if (type == W2)
    {
        tx_reg[2] = data & 0xff;
        tx_reg[3] = (data >> 8) & 0xff;
        spi_write_register(0x3E, tx_reg, 4, rxdata);
        HAL_Delay(1);
        tx_buffer[0] = checksum(tx_reg, 4);
        tx_buffer[1] = 0x06;
        spi_write_register(0x60, tx_buffer, 2, rxdata);
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
void bq769x2_set_register(uint16_t reg_addr, uint32_t reg_data, uint8_t datalen)
{
    uint8_t tx_buffer[2]   = {0};
    uint8_t tx_reg_data[6] = {0};

    tx_reg_data[0] = reg_addr & 0xff;
    tx_reg_data[1] = (reg_addr >> 8) & 0xff;
    tx_reg_data[2] = reg_data & 0xff;

    switch (datalen)
    {
        case 1:
            spi_write_register(0x3E, tx_reg_data, 3, rxdata);
            HAL_Delay(2);
            tx_buffer[0] = checksum(tx_reg_data, 3);
            tx_buffer[1] = 0x05;
            spi_write_register(0x60, tx_buffer, 2, rxdata);
            HAL_Delay(2);
            break;

        case 2:
            tx_reg_data[3] = (reg_data >> 8) & 0xff;
            spi_write_register(0x3E, tx_reg_data, 4, rxdata);
            HAL_Delay(2);
            tx_buffer[0] = checksum(tx_reg_data, 4);
            tx_buffer[1] = 0x06;
            spi_write_register(0x60, tx_buffer, 2, rxdata);
            HAL_Delay(2);
            break;

        case 4:
            tx_reg_data[3] = (reg_data >> 8) & 0xff;
            tx_reg_data[4] = (reg_data >> 16) & 0xff;
            tx_reg_data[5] = (reg_data >> 24) & 0xff;
            spi_write_register(0x3E, tx_reg_data, 6, rxdata);
            HAL_Delay(2);
            tx_buffer[0] = checksum(tx_reg_data, 6);
            tx_buffer[1] = 0x08;
            spi_write_register(0x60, tx_buffer, 2, rxdata);
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
uint16_t bq769x2_read_voltage(uint8_t command)
{
    direct_commands(command, 0x00, R);

    // Cells 1 through 16 (0x14 to 0x32)
    if (command >= ADDR_CELL_VOLTAGES && command <= (ADDR_CELL_VOLTAGES + (15 * 2)))
    {
        return (rx_32byte[1] * 256 + rx_32byte[0]);    // voltage is reported in mV
    }
    else
    {
        // voltage is reported in 0.01V (centivolts) units by default,
        //(Settings:Configuration:DA Configuration USER_VOLTS_CV in TRM)
        return 10 * (rx_32byte[1] * 256 + rx_32byte[0]);
    }
}

/**
 * @brief Reads cell voltages from BQ769x2 based on BMS model configuration.
 *
 * For BMS_MODEL_10S: Reads cells 1-9 consecutively, cell 10 from address 16.
 * For BMS_MODEL_14S: Reads cells 1-13 consecutively, cell 14 from address 16.
 * The last cell is always read from the Cell 16 address due to BQ769x2 wiring.
 */
void bq769x2_read_cell_voltages(void)
{
    uint8_t cell_addr = ADDR_CELL_VOLTAGES; /* Cell1Voltage is 0x14 */

    /* Read consecutive cells (1 to N-1) */
    for (uint8_t cell = 0; cell < BMS_CONSECUTIVE_CELLS; cell++)
    {
        cell_voltage[cell] = bq769x2_read_voltage(cell_addr);
        cell_addr += 2;
    }

    /* Read last cell from Cell 16 address */
    cell_addr = ADDR_CELL_VOLTAGES + (BMS_LAST_CELL_INDEX * 2);

    cell_voltage[BMS_CELL_COUNT - 1] = bq769x2_read_voltage(cell_addr);
}

void bq769x2_readall_voltages(void)
{
    int cellvoltageholder = ADDR_CELL_VOLTAGES;    // Cell1Voltage is 0x14

    volatile uint16_t allvoltages[16];

    for (int x = 0; x < 16; x++)
    {
        allvoltages[x] = bq769x2_read_voltage(cellvoltageholder);
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
int16_t bq769x2_read_current()
{
    direct_commands(ADDR_CC2_CURRENT, 0x00, R);
    return (rx_32byte[1] * 256 + rx_32byte[0]);    // current is reported in mA
}

/**
 * @brief Reads temperature from BQ769x2 device for a given command address.
 *
 * Converts the raw temperature value from 0.1 Kelvin units to Celsius.
 *
 * @param command Command address to read temperature from.
 * @return float Temperature in degrees Celsius.
 */
float bq769x2_read_temperature(uint8_t command)
{
    direct_commands(command, 0x00, R);

    return (0.1 * (float)(rx_32byte[1] * 256 + rx_32byte[0])) - 273.15;    // converts from 0.1K to Celcius
}

/**
 * @brief Reads the Safety Status registers (A/B/C) from the BQ769x2 device.
 *
 * This function checks which primary protections have been triggered by reading
 * the Safety Status A and B registers. It extracts Overvoltage (OV),
 * Undervoltage (UV), and Overcurrent (OC) fault bits. Overtemperature (OT)
 * faults are checked from Safety Status B. If any protection is triggered,
 * the protections_triggered flag is set.
 */
void bq769x2_read_safety_status()
{
    direct_commands(ADDR_SAFETY_STATUS_A, 0x00, R);
    value_safety_status_a = rx_32byte[0];

    UV_fault = (0x4 & rx_32byte[0]) >> 2;
    OV_fault = (0x8 & rx_32byte[0]) >> 3;
    // OCC_Fault = (0x10 & rx_32byte[0]) >> 4;
    // OCD1_Fault = (0x20 & rx_32byte[0]) >> 5;
    // OCD2_Fault = (0x40 & rx_32byte[0]) >> 6;
    // SCD_Fault = (0x80 & rx_32byte[0]) >> 7;

    // check if any over-current bits are set
    if ((rx_32byte[0] & 0xF0) != 0)
    {
        OC_fault = 1;
    }
    else
    {
        OC_fault = 0;
    }

    direct_commands(ADDR_SAFETY_STATUS_B, 0x00, R);
    value_safety_status_b = rx_32byte[0];

    // check if any over-temperature bits are set
    if ((rx_32byte[0] & 0xF0) != 0)
    {
        OT_fault = 1;
    }
    else
    {
        OT_fault = 0;
    }

    /* Safety Status C needed? */
    direct_commands(ADDR_SAFETY_STATUS_C, 0x00, R);
    value_safety_status_c = rx_32byte[0];

    if ((value_safety_status_a + value_safety_status_b + value_safety_status_c) > 0)
    {
        protections_triggered = 1;
    }
    else
    {
        protections_triggered = 0;
    }

    /* Read safety alert statuses */
    direct_commands(0x02, 0, R);    // safety alert A
    direct_commands(0x04, 0, R);    // safety alert B
    direct_commands(0x06, 0, R);    // safety alert C
}

/**
 * @brief Reads the Permanent Fail Status registers (A/B/C) from the BQ769x2.
 *
 * This function checks which permanent failures have been triggered by reading
 * the Permanent Fail Status A, B, and C registers. If any permanent fault is
 * detected, the permanent_fault_triggered flag is set.
 */
void bq769x2_read_pf_status()
{
    direct_commands(ADDR_PF_STATUS_A, 0x00, R);
    value_pf_status_a = rx_32byte[0];

    direct_commands(ADDR_PF_STATUS_B, 0x00, R);
    value_pf_status_b = rx_32byte[0];

    direct_commands(ADDR_PF_STATUS_C, 0x00, R);
    value_pf_status_c = rx_32byte[0];

    if ((value_pf_status_a + value_pf_status_b + value_pf_status_c) > 0)
    {
        permanent_fault_triggered = 1;
    }
    else
    {
        permanent_fault_triggered = 0;
    }

    direct_commands(0x11, 0, R);    // pf status D

    direct_commands(0x0A, 0, R);    // pf alert A
    direct_commands(0x0C, 0, R);    // pf alert B
    direct_commands(0x0E, 0, R);    // pf alert C
    direct_commands(0x10, 0, R);    // pf alert D
}

/**
 * @brief Reads the Alarm Status register from the BQ769x2 device.
 *
 * @return uint16_t Value of the Alarm Status register.
 */
uint16_t bq769x2_read_alarm_status()
{
    direct_commands(ADDR_ALARM_STATUS, 0x00, R);
    return (rx_32byte[1] * 256 + rx_32byte[0]);
}

/**
 * @brief Reads the FET Status register from the BQ769x2 device.
 *
 * This function checks which FETs (DSG and CHG) are enabled by reading the FET
 * Status register and updates the corresponding status flags.
 */
void bq769x2_read_fet_status()
{
    direct_commands(ADDR_FET_STATUS, 0x00, R);
    FET_status = rx_32byte[0];

    CHG = (0x1 & rx_32byte[0]);
    DSG = (0x4 & rx_32byte[0]) >> 2;
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

    subcommands(REG12_CONFIG, 0, R);
    if (rx_32byte[0] != SPI_BMS_REG12_CONFIG)
    {
        if (memcmp(rx_32byte, EMPTY_BUFFER, 4) == 0 &&
            memcmp(rxdata, EMPTY_BUFFER, 4) == 0)
        {
            return BMS_OTP_BQ_NOT_DETECTED;
        }

        direct_commands(ADDR_BATTERY_STATUS, 0, R);
        if ((rx_32byte[1] & 0x03) != 0x01)
        {
            /* Enter FULLACCESS mode sequence */
            /* By default device is in FULLACCESS */
            return BMS_OTP_ERROR;
        }

        command_subcommands(ADDR_SET_CFGUPDATE);
        bq769x2_set_register(REG12_CONFIG, SPI_BMS_REG12_CONFIG, 1);    // REG2 @ 3.3V & REG1 @ 5V
        command_subcommands(ADDR_EXIT_CFGUPDATE);

        subcommands(REG12_CONFIG, 0, R);
        if (rx_32byte[0] != SPI_BMS_REG12_CONFIG)
        {
            return BMS_OTP_NOT_PROGRAMMED;
        }

        command_subcommands(ADDR_SET_CFGUPDATE);
        direct_commands(ADDR_BATTERY_STATUS, 0, R);
        if ((rx_32byte[0] & 0x80) >> 7)
        {
            // Conditions for OTP programming NOT met
            subcommands(ADDR_OTP_WR_CHECK, 0x0000, R);    // check failed condition
            if (rx_32byte[0] & 0x01)
            {
                // stack voltage is above the allowed OTP programming voltage
                HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 3722);    // ~3V
            }
            else if (rx_32byte[0] & 0x02)
            {
                // stack voltage is below the allowed OTP programming voltage
                HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2480);    // ~2V
            }
            else if (rx_32byte[0] & 0x04)
            {
                // internal temperature is above the allowed OTP programming temperature range
                HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 1240);    // ~1V
            }
            else
            {
                HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
            }

            command_subcommands(ADDR_EXIT_CFGUPDATE);
            return BMS_OTP_CONDITIONS_NOT_MET;
        }

        subcommands(ADDR_OTP_WR_CHECK, 0x0000, R);

        uint8_t txreg[2];
        txreg[0] = ADDR_OTP_WR_CHECK & 0xff;
        txreg[1] = (ADDR_OTP_WR_CHECK >> 8) & 0xff;
        spi_write_register(0x3E, txreg, 2, rxdata);
        HAL_Delay(100);
        spi_read_register(0x40, rx_32byte, 32, rxdata);
        if (rx_32byte[0] != 0x80)
        {
            // OTP programming NOT possible
            command_subcommands(ADDR_EXIT_CFGUPDATE);
            return BMS_OTP_NOT_PROGRAMMED;
        }

        txreg[0] = ADDR_OTP_WRITE & 0xff;
        txreg[1] = (ADDR_OTP_WRITE >> 8) & 0xff;
        spi_write_register(0x3E, txreg, 2, rxdata);
        HAL_Delay(200);
        spi_read_register(0x40, rx_32byte, 32, rxdata);

        if (rx_32byte[0] != 0x80)
        {
            // OTP programming FAILED
            return BMS_OTP_FAILED;
        }
        command_subcommands(ADDR_EXIT_CFGUPDATE);

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
 * @brief Returns the smallest cell voltage from the cell_voltage array.
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
        if (cell_voltage[cell] < smallest_voltage)
        {
            smallest_voltage = cell_voltage[cell];
        }
    }

    return smallest_voltage;
}

/**
 * @brief Returns the largest cell voltage from the cell_voltage array.
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
        if (cell_voltage[cell] > largest_voltage)
        {
            largest_voltage = cell_voltage[cell];
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

    if (protections_triggered || permanent_fault_triggered)
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

    subcommands(ADDR_CB_ACTIVE_CELLS, 0, R);

    uint16_t balancing_active_cells = (rx_32byte[1] * 256 + rx_32byte[0]);
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
        uint16_t stack_voltage = bq769x2_read_voltage(ADDR_STACK_VOLTAGE);
        uint16_t pack_voltage  = bq769x2_read_voltage(ADDR_PACK_PIN_VOLTAGE);

        if (stack_voltage < pack_voltage)
        {
            charging_status = 1;
        }
    }
    else
    {
        /* FETs on */
        int16_t current = bq769x2_read_current();
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

    if (protections_triggered || permanent_fault_triggered)
    {
        faults = (PF_fault << 4) | (OC_fault << 3) | (OT_fault << 2) | (UV_fault << 1) | OV_fault;
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
unsigned char checksum(unsigned char *ptr, unsigned char len)
// The checksum is the inverse of the sum of the bytes.
{
    unsigned char i;
    unsigned char checksum = 0;

    for (i = 0; i < len; i++)
    {
        checksum += ptr[i];
    }

    checksum = 0xFF & ~checksum;

    return checksum;
}
