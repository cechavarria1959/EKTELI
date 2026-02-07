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
#include <string.h>
#include "bms.h"
#include "main.h"
#include "spi.h"
#include "stm32l4xx_hal.h"


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

    // Exit CONFIGUPDATE mode  - Subcommand 0x0092
    CommandSubcommands(ADDR_EXIT_CFGUPDATE);
}

void DirectCommands(uint8_t command, uint16_t data, uint8_t type)
// See the TRM or the BQ76952 header file for a full list of Direct Commands
{    // type: R = read, W = write
    uint8_t TX_data[2] = {0x00};

    // little endian format
    TX_data[0] = data & 0xff;
    TX_data[1] = (data >> 8) & 0xff;

    if (type == R)
    {                                       // Read
        SPI_ReadReg(command, rxdata, 2);    // RX_data is a global variable
        HAL_Delay(2);
    }
    if (type == W)
    {    // write
        // Control_status, alarm_status, alarm_enable all 2 bytes long
        SPI_WriteReg(command, TX_data, 2);
        HAL_Delay(2);
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
    HAL_Delay(2);
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
        HAL_Delay(2);
        SPI_ReadReg(0x40, RX_32Byte, 32);    // RX_32Byte is a global variable
    }
    else if (type == W)
    {
        // FET_Control, REG12_Control
        TX_Reg[2] = data & 0xff;
        SPI_WriteReg(0x3E, TX_Reg, 3);
        HAL_Delay(1);
        TX_Buffer[0] = Checksum(TX_Reg, 3);
        TX_Buffer[1] = 0x05;    // combined length of registers address and data
        SPI_WriteReg(0x60, TX_Buffer, 2);
        HAL_Delay(1);
    }
    else if (type == W2)
    {    // write data with 2 bytes
        // CB_Active_Cells, CB_SET_LVL
        TX_Reg[2] = data & 0xff;
        TX_Reg[3] = (data >> 8) & 0xff;
        SPI_WriteReg(0x3E, TX_Reg, 4);
        HAL_Delay(1);
        TX_Buffer[0] = Checksum(TX_Reg, 4);
        TX_Buffer[1] = 0x06;    // combined length of registers address and data
        SPI_WriteReg(0x60, TX_Buffer, 2);
        HAL_Delay(1);
    }
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
            HAL_Delay(2);
            TX_Buffer[0] = Checksum(TX_RegData, 3);
            TX_Buffer[1] = 0x05;                 // combined length of register address and data
            SPI_WriteReg(0x60, TX_Buffer, 2);    // Write the checksum and length
            HAL_Delay(2);
            break;
        case 2:    // 2 byte datalength
            TX_RegData[3] = (reg_data >> 8) & 0xff;
            SPI_WriteReg(0x3E, TX_RegData, 4);
            HAL_Delay(2);
            TX_Buffer[0] = Checksum(TX_RegData, 4);
            TX_Buffer[1] = 0x06;                 // combined length of register address and data
            SPI_WriteReg(0x60, TX_Buffer, 2);    // Write the checksum and length
            HAL_Delay(2);
            break;
        case 4:    // 4 byte datalength, Only used for CCGain and Capacity Gain
            TX_RegData[3] = (reg_data >> 8) & 0xff;
            TX_RegData[4] = (reg_data >> 16) & 0xff;
            TX_RegData[5] = (reg_data >> 24) & 0xff;
            SPI_WriteReg(0x3E, TX_RegData, 6);
            HAL_Delay(2);
            TX_Buffer[0] = Checksum(TX_RegData, 6);
            TX_Buffer[1] = 0x08;                 // combined length of register address and data
            SPI_WriteReg(0x60, TX_Buffer, 2);    // Write the checksum and length
            HAL_Delay(2);
            break;
    }
}

uint16_t BQ769x2_ReadVoltage(uint8_t command)
// This function can be used to read a specific cell voltage or stack / pack / LD voltage
{
    // RX_data is global var
    DirectCommands(command, 0x00, R);

    if (command >= ADDR_CELL_VOLTAGES && command <= (ADDR_CELL_VOLTAGES + (15 * 2)))    // Cells 1 through 16 (0x14 to 0x32)
    {
        return (rxdata[1] * 256 + rxdata[0]);    // voltage is reported in mV
    }
    else
    {
        // voltage is reported in 0.01V (centivolts) units by default,
        //(Settings:Configuration:DA Configuration USER_VOLTS_CV in TRM)
        return 10 * (rxdata[1] * 256 + rxdata[0]);
    }
}

void BQ769x2_Readcell_voltages(void)
{
    int cellvoltageholder = ADDR_CELL_VOLTAGES;    // Cell1Voltage is 0x14
    for (int x = 0; x < 16; x++)
    {    // Reads all cell voltages
        CellVoltage[x] = BQ769x2_ReadVoltage(cellvoltageholder);
        cellvoltageholder += 2;
    }
}

int16_t BQ769x2_ReadCurrent()
// Reads PACK current
{
    DirectCommands(0x3A, 0x00, R);
    return (rxdata[1] * 256 + rxdata[0]);    // current is reported in mA
}

float BQ769x2_ReadTemperature(uint8_t command)
{
    DirectCommands(command, 0x00, R);
    // RX_data is a global var
    return (0.1 * (float)(rxdata[1] * 256 + rxdata[0])) - 273.15;    // converts from 0.1K to Celcius
}

void BQ769x2_ReadSafetyStatus()
{    // good example functions
    // Read Safety Status A/B/C and find which bits are set
    // This shows which primary protections have been triggered
    DirectCommands(ADDR_SAFETY_STATUS_A, 0x00, R);
    value_SafetyStatusA = (rxdata[1] * 256 + rxdata[0]);
    // Example Fault Flags
    OV_Fault = ((0x8 & rxdata[0]) >> 3);
    UV_Fault = ((0x4 & rxdata[0]) >> 2);
    // SCD_Fault = ((0x8 & rxdata[1])>>3);
    // OCD_Fault = ((0x2 & rxdata[1])>>1);
    if ((rxdata[0] & 0xF0) != 0)    // check if any over-current bits are set
    {
        OC_Fault = 1;
    }
    else
    {
        OC_Fault = 0;
    }

    DirectCommands(ADDR_SAFETY_STATUS_B, 0x00, R);
    value_SafetyStatusB = (rxdata[1] * 256 + rxdata[0]);
    if ((rxdata[0] & 0xF0) != 0)    // check if any over-temperature bits are set
    {
        OT_Fault = 1;
    }
    else
    {
        OT_Fault = 0;
    }

    /* Safety Status C needed? */
    // DirectCommands(ADDR_SAFETY_STATUS_C, 0x00, R);
    // value_SafetyStatusC = (rxdata[1] * 256 + rxdata[0]);

    if ((value_SafetyStatusA + value_SafetyStatusB + value_SafetyStatusC) > 0)
    {
        ProtectionsTriggered = 1;
    }
    else
    {
        ProtectionsTriggered = 0;
    }
}

void BQ769x2_ReadPFStatus()
{
    // Read Permanent Fail Status A/B/C and find which bits are set
    // This shows which permanent failures have been triggered
    DirectCommands(ADDR_PF_STATUS_A, 0x00, R);
    value_PFStatusA = (rxdata[1] * 256 + rxdata[0]);
    DirectCommands(ADDR_PF_STATUS_B, 0x00, R);
    value_PFStatusB = (rxdata[1] * 256 + rxdata[0]);
    DirectCommands(ADDR_PF_STATUS_C, 0x00, R);
    value_PFStatusC = (rxdata[1] * 256 + rxdata[0]);

    if ((value_PFStatusA + value_PFStatusB + value_PFStatusC) > 0)
    {
        PermanentFaultTriggered = 1;
    }
    else
    {
        PermanentFaultTriggered = 0;
    }
}

uint16_t BQ769x2_ReadAlarmStatus()
{
    // Read this register to find out why the ALERT pin was asserted
    DirectCommands(ADDR_ALARM_STATUS, 0x00, R);
    return (rxdata[1] * 256 + rxdata[0]);
}

void BQ769x2_ReadFETStatus()
{
    // Read FET Status to see which FETs are enabled
    DirectCommands(ADDR_FET_STATUS, 0x00, R);
    FET_Status = (rxdata[1] * 256 + rxdata[0]);

    DSG = ((0x4 & rxdata[0]) >> 2);    // discharge FET state
    CHG = (0x1 & rxdata[0]);           // charge FET state
}

/* mainly checks REG2 Status for MCU power rail, which is off for
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
        if ((rxdata[1] & 0x03) != 0x01)
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
        if ((rxdata[0] & 0x80) >> 7)
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
        SPI_WriteReg(0x3E, txreg, 2);
        HAL_Delay(100);
        SPI_ReadReg(0x40, RX_32Byte, 32);
        if (RX_32Byte[0] != 0x80)
        {
            // OTP programming NOT possible
            CommandSubcommands(ADDR_EXIT_CFGUPDATE);
            return BMS_OTP_NOT_PROGRAMMED;
        }

        txreg[0] = ADDR_OTP_WRITE & 0xff;
        txreg[1] = (ADDR_OTP_WRITE >> 8) & 0xff;
        SPI_WriteReg(0x3E, txreg, 2);
        HAL_Delay(200);
        SPI_ReadReg(0x40, RX_32Byte, 32);

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

uint16_t get_smallest_cell_voltage()
{
    uint16_t smallest_voltage = 0xFFFF;
    for (int i = 0; i < 16; i++)
    {
        if (CellVoltage[i] < smallest_voltage)
        {
            smallest_voltage = CellVoltage[i];
        }
    }

    return smallest_voltage;
}

uint16_t get_largest_cell_voltage()
{
    uint16_t largest_voltage = 0x0000;
    for (int i = 0; i < 16; i++)
    {
        if (CellVoltage[i] > largest_voltage)
        {
            largest_voltage = CellVoltage[i];
        }
    }

    return largest_voltage;
}

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

uint8_t get_balancing_status(void)
{
    uint8_t balancing_status = 0u;

    Subcommands(ADDR_CB_ACTIVE_CELLS, 0, R);

    uint16_t balancing_active_cells = (RX_32Byte[1] * 256 + RX_32Byte[0]);
    if (balancing_active_cells != 0xFFFF || balancing_active_cells != 0x0000)
    {
        balancing_status = 1;
    }
    else
    {
        balancing_status = 0;
    }

    return balancing_status;
}

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
        if (current < 0)
        {
            charging_status = 1;
        }
    }

    return charging_status;
}

uint8_t bms_get_faults(void)
{
    uint8_t faults = 0;

    if (ProtectionsTriggered || PermanentFaultTriggered)
    {
        faults = (PF_Fault << 4) | (OC_Fault << 3) | (OT_Fault << 2) | (UV_Fault << 1) | OV_Fault;
    }

    return faults;
}

uint8_t get_fet_status(void)
{
    return (CHG << 1) | DSG;
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


/* Private user code ---------------------------------------------------------*/
unsigned char Checksum(unsigned char *ptr, unsigned char len)
// Calculates the checksum when writing to a RAM register. The checksum is the inverse of the sum of the bytes.
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
