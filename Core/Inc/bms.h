/**
 * @file    bms.h
 * @brief   Battery Management System (BMS) interface definitions.
 * @details Contains enums, constants, and function prototypes for
 *          communication and control of the BQ769x2 battery management
 *          IC. Provides register addresses, command timings, and status
 *          types used throughout the firmware for battery monitoring,
 *          protection, and balancing operations.
 *
 * @author  CesarO
 * @date    2025-10-07
 *
 * @copyright Copyright (c) 2025 Ekteli.
 */

#ifndef INC_BMS_H_
#define INC_BMS_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>


/* Exported types ------------------------------------------------------------*/
typedef enum
{
    /* Direct Commands (1 byte command) */
    ADDR_CONTROL_STATUS       = 0x00,
    ADDR_SAFETY_ALERT_STATUS  = 0x02,    // hasta 0x07
    ADDR_SAFETY_STATUS_A      = 0x03,
    ADDR_SAFETY_STATUS_B      = 0x05,
    ADDR_SAFETY_STATUS_C      = 0x07,
    ADDR_PF_ALERT_STATUS      = 0x0A,    // hasta 0x11
    ADDR_PF_STATUS_A          = 0x0B,
    ADDR_PF_STATUS_B          = 0x0D,
    ADDR_PF_STATUS_C          = 0x0F,
    ADDR_BATTERY_STATUS       = 0x12,
    ADDR_CELL_VOLTAGES        = 0x14,    // hasta 0x32
    ADDR_STACK_VOLTAGE        = 0x34,
    ADDR_PACK_PIN_VOLTAGE     = 0x36,
    ADDR_LD_PIN_VOLTAGE       = 0x38,
    ADDR_CC2_CURRENT          = 0x3A,
    ADDR_ALARM_STATUS         = 0x62,
    ADDR_ALARM_RAW_STATUS     = 0x64,
    ADDR_ALARM_ENABLE         = 0x66,
    ADDR_INTERNAL_TEMP        = 0x68,
    ADDR_THERMISTOR_TEMP      = 0x6A,    // hasta 0x7A
    ADDR_FET_STATUS           = 0x7F,

    /* Subcommands (2 bytes commands) */
    ADDR_DEVICE_NUMBER        = 0x0001,
    ADDR_FW_VERSION           = 0x0002,
    ADDR_HW_VERSION           = 0x0003,
    ADDR_IROM_SIG             = 0x0004,
    ADDR_STATIC_CFG_SIG       = 0x0005,
    ADDR_DROM_SIG             = 0x0009,
    ADDR_EXIT_DEEPSLEEP       = 0x000E,
    ADDR_DEEPSLEEP            = 0x000F,
    ADDR_SHUTDOWN             = 0x0010,
    ADDR_RESET                = 0x0012,
    ADDR_PDSGTEST             = 0x001C,
    ADDR_FUSE_TOGGLE          = 0x001D,
    ADDR_PCHGTEST             = 0x001E,
    ADDR_CHGTEST              = 0x001F,
    ADDR_DSGTEST              = 0x0020,
    ADDR_FET_ENABLE           = 0x0022,
    ADDR_PF_ENABLE            = 0x0024,
    ADDR_SEAL                 = 0x0030,
    ADDR_SAVED_PF_STATUS      = 0x0053,
    ADDR_MANUFACTURING_STATUS = 0x0057,
    ADDR_MANU_DATA            = 0x0070,
    ADDR_DASTATUS1            = 0x0071,
    ADDR_DASTATUS2            = 0x0072,
    ADDR_DASTATUS3            = 0x0073,
    ADDR_DASTATUS4            = 0x0074,
    ADDR_DASTATUS5            = 0x0075,
    ADDR_DASTATUS6            = 0x0076,
    ADDR_DASTATUS7            = 0x0077,
    ADDR_CUV_SNAPSHOT         = 0x0080,
    ADDR_COV_SNAPSHOT         = 0x0081,
    ADDR_RESET_PASSQ          = 0x0082,
    ADDR_CB_ACTIVE_CELLS      = 0x0083,
    ADDR_CB_SET_LVL1          = 0x0084,
    ADDR_CBSTATUS1            = 0x0085,
    ADDR_CBSTATUS2            = 0x0086,
    ADDR_CBSTATUS3            = 0x0087,
    ADDR_PTO_RECOVER          = 0x008A,
    ADDR_SET_CFGUPDATE        = 0x0090,
    ADDR_EXIT_CFGUPDATE       = 0x0092,
    ADDR_DSG_PDSG_OFF         = 0x0093,
    ADDR_CHG_PCHG_OFF         = 0x0094,
    ADDR_ALL_FETS_OFF         = 0x0095,
    ADDR_ALL_FETS_ON          = 0x0096,
    ADDR_FET_CONTROL          = 0x0097,
    ADDR_REG12_CONTROL        = 0x0098,
    ADDR_SLEEP_ENABLE         = 0x0099,
    ADDR_SLEEP_DISABLE        = 0x009A,
    ADDR_OCDL_RECOVER         = 0x009B,
    ADDR_SCDL_RECOVER         = 0x009C,
    ADDR_LOAD_DET_RESTART     = 0x009D,
    ADDR_LOAD_DET_ON          = 0x009E,
    ADDR_LOAD_DET_OFF         = 0x009F,
    ADDR_OTP_WR_CHECK         = 0x00A0,
    ADDR_OTP_WRITE            = 0x00A1,
    ADDR_GPO_SUBCOMMANDS      = 0x2800,    // hasta 0x2818
    ADDR_PF_FORCE_A           = 0x2857,
    ADDR_PF_FORCE_B           = 0x29A3,
    ADDR_SWAP_COMM_MODE       = 0x29BC,
    ADDR_SWAP_TO_I2C          = 0x29E7,
    ADDR_SWAP_TO_SPI          = 0x7C35,
    ADDR_SWAP_TO_HDQ          = 0x7C40,
    ADDR_READ_CAL1            = 0xF081
} bms_command_address_t;

typedef enum
{
    CELL1_GAIN                  = 0x9180,    // Calibration:Voltage:Cell 1 Gain
    CELL2_GAIN                  = 0x9182,    // Calibration:Voltage:Cell 2 Gain
    CELL3_GAIN                  = 0x9184,    // Calibration:Voltage:Cell 3 Gain
    CELL4_GAIN                  = 0x9186,    // Calibration:Voltage:Cell 4 Gain
    CELL5_GAIN                  = 0x9188,    // Calibration:Voltage:Cell 5 Gain
    CELL6_GAIN                  = 0x918A,    // Calibration:Voltage:Cell 6 Gain
    CELL7_GAIN                  = 0x918C,    // Calibration:Voltage:Cell 7 Gain
    CELL8_GAIN                  = 0x918E,    // Calibration:Voltage:Cell 8 Gain
    CELL9_GAIN                  = 0x9190,    // Calibration:Voltage:Cell 9 Gain
    CELL10_GAIN                 = 0x9192,    // Calibration:Voltage:Cell 10 Gain
    CELL11_GAIN                 = 0x9194,    // Calibration:Voltage:Cell 11 Gain
    CELL12_GAIN                 = 0x9196,    // Calibration:Voltage:Cell 12 Gain
    CELL13_GAIN                 = 0x9198,    // Calibration:Voltage:Cell 13 Gain
    CELL14_GAIN                 = 0x919A,    // Calibration:Voltage:Cell 14 Gain
    CELL15_GAIN                 = 0x919C,    // Calibration:Voltage:Cell 15 Gain
    CELL16_GAIN                 = 0x919E,    // Calibration:Voltage:Cell 16 Gain
    PACK_GAIN                   = 0x91A0,    // Calibration:Voltage:Pack Gain
    TOS_GAIN                    = 0x91A2,    // Calibration:Voltage:TOS Gain
    LD_GAIN                     = 0x91A4,    // Calibration:Voltage:LD Gain
    ADC_GAIN                    = 0x91A6,    // Calibration:Voltage:ADC Gain
    CC_GAIN                     = 0x91A8,    // Calibration:Current:CC Gain
    CAPACITY_GAIN               = 0x91AC,    // Calibration:Current:Capacity Gain
    VCELL_OFFSET                = 0x91B0,    // Calibration:Vcell Offset:Vcell Offset
    VDIV_OFFSET                 = 0x91B2,    // Calibration:V Divider Offset:Vdiv Offset
    COULOMB_COUNTER_OFFSET      = 0x91C6,    // Calibration:Current Offset:Coulomb Counter Offset Samples
    BOARD_OFFSET                = 0x91C8,    // Calibration:Current Offset:Board Offset
    INTERNAL_TEMP_OFFSET        = 0x91CA,    // Calibration:Temperature:Internal Temp Offset
    CFETOFF_TEMP_OFFSET         = 0x91CB,    // Calibration:Temperature:CFETOFF Temp Offset
    DFETOFF_TEMP_OFFSET         = 0x91CC,    // Calibration:Temperature:DFETOFF Temp Offset
    ALERT_TEMP_OFFSET           = 0x91CD,    // Calibration:Temperature:ALERT Temp Offset
    TS1_TEMP_OFFSET             = 0x91CE,    // Calibration:Temperature:TS1 Temp Offset
    TS2_TEMP_OFFSET             = 0x91CF,    // Calibration:Temperature:TS2 Temp Offset
    TS3_TEMP_OFFSET             = 0x91D0,    // Calibration:Temperature:TS3 Temp Offset
    HDQ_TEMP_OFFSET             = 0x91D1,    // Calibration:Temperature:HDQ Temp Offset
    DCHG_TEMP_OFFSET            = 0x91D2,    // Calibration:Temperature:DCHG Temp Offset
    DDSG_TEMP_OFFSET            = 0x91D3,    // Calibration:Temperature:DDSG Temp Offset
    INT_GAIN                    = 0x91E2,    // Calibration:Internal Temp Model:Int Gain
    INT_BASE_OFFSET             = 0x91E4,    // Calibration:Internal Temp Model:Int base offset
    INT_MAXIMUM_AD              = 0x91E6,    // Calibration:Internal Temp Model:Int Maximum AD
    INT_MAXIMUM_TEMP            = 0x91E8,    // Calibration:Internal Temp Model:Int Maximum Temp
    T18K_COEFF_A1               = 0x91EA,    // Calibration:18K Temperature Model:Coeff a1
    T18K_COEFF_A2               = 0x91EC,    // Calibration:18K Temperature Model:Coeff a2
    T18K_COEFF_A3               = 0x91EE,    // Calibration:18K Temperature Model:Coeff a3
    T18K_COEFF_A4               = 0x91F0,    // Calibration:18K Temperature Model:Coeff a4
    T18K_COEFF_A5               = 0x91F2,    // Calibration:18K Temperature Model:Coeff a5
    T18K_COEFF_B1               = 0x91F4,    // Calibration:18K Temperature Model:Coeff b1
    T18K_COEFF_B2               = 0x91F6,    // Calibration:18K Temperature Model:Coeff b2
    T18K_COEFF_B3               = 0x91F8,    // Calibration:18K Temperature Model:Coeff b3
    T18K_COEFF_B4               = 0x91FA,    // Calibration:18K Temperature Model:Coeff b4
    T18K_ADC0                   = 0x91FE,    // Calibration:18K Temperature Model:Adc0
    T180K_COEFF_A1              = 0x9200,    // Calibration:180K Temperature Model:Coeff a1
    T180K_COEFF_A2              = 0x9202,    // Calibration:180K Temperature Model:Coeff a2
    T180K_COEFF_A3              = 0x9204,    // Calibration:180K Temperature Model:Coeff a3
    T180K_COEFF_A4              = 0x9206,    // Calibration:180K Temperature Model:Coeff a4
    T180K_COEFF_A5              = 0x9208,    // Calibration:180K Temperature Model:Coeff a5
    T180K_COEFF_B1              = 0x920A,    // Calibration:180K Temperature Model:Coeff b1
    T180K_COEFF_B2              = 0x920C,    // Calibration:180K Temperature Model:Coeff b2
    T180K_COEFF_B3              = 0x920E,    // Calibration:180K Temperature Model:Coeff b3
    T180K_COEFF_B4              = 0x9210,    // Calibration:180K Temperature Model:Coeff b4
    T180K_ADC0                  = 0x9214,    // Calibration:180K Temperature Model:Adc0
    CUSTOM_COEFF_A1             = 0x9216,    // Calibration:Custom Temperature Model:Coeff a1
    CUSTOM_COEFF_A2             = 0x9218,    // Calibration:Custom Temperature Model:Coeff a2
    CUSTOM_COEFF_A3             = 0x921A,    // Calibration:Custom Temperature Model:Coeff a3
    CUSTOM_COEFF_A4             = 0x921C,    // Calibration:Custom Temperature Model:Coeff a4
    CUSTOM_COEFF_A5             = 0x921E,    // Calibration:Custom Temperature Model:Coeff a5
    CUSTOM_COEFF_B1             = 0x9220,    // Calibration:Custom Temperature Model:Coeff b1
    CUSTOM_COEFF_B2             = 0x9222,    // Calibration:Custom Temperature Model:Coeff b2
    CUSTOM_COEFF_B3             = 0x9224,    // Calibration:Custom Temperature Model:Coeff b3
    CUSTOM_COEFF_B4             = 0x9226,    // Calibration:Custom Temperature Model:Coeff b4
    CUSTOM_RC0                  = 0x9228,    // Calibration:Custom Temperature Model:Rc0
    CUSTOM_ADC0                 = 0x922A,    // Calibration:Custom Temperature Model:Adc0
    COULOMB_COUNTER_DEADBAND    = 0x922D,    // Calibration:Current Deadband:Coulomb Counter Deadband
    CUV_THRESHOLD_OVERRIDE      = 0x91D4,    // Calibration:CUV:CUV Threshold Override
    COV_THRESHOLD_OVERRIDE      = 0x91D6,    // Calibration:COV:COV Threshold Override
    MIN_BLOW_FUSE_VOLTAGE       = 0x9231,    // Settings:Fuse:Min Blow Fuse Voltage
    FUSE_BLOW_TIMEOUT           = 0x9233,    // Settings:Fuse:Fuse Blow Timeout
    POWER_CONFIG                = 0x9234,    // Settings:Configuration:Power Config
    REG12_CONFIG                = 0x9236,    // Settings:Configuration:REG12 Config
    REG0_CONFIG                 = 0x9237,    // Settings:Configuration:REG0 Config
    HWD_REGULATOR_OPTIONS       = 0x9238,    // Settings:Configuration:HWD Regulator Options
    COMM_TYPE                   = 0x9239,    // Settings:Configuration:Comm Type
    I2C_ADDRESS                 = 0x923A,    // Settings:Configuration:I2C Address
    SPI_CONFIGURATION           = 0x923C,    // Settings:Configuration:SPI Configuration
    COMM_IDLE_TIME              = 0x923D,    // Settings:Configuration:Comm Idle Time
    CFETOFF_PIN_CONFIG          = 0x92FA,    // Settings:Configuration:CFETOFF Pin Config
    DFETOFF_PIN_CONFIG          = 0x92FB,    // Settings:Configuration:DFETOFF Pin Config
    ALERT_PIN_CONFIG            = 0x92FC,    // Settings:Configuration:ALERT Pin Config
    TS1_CONFIG                  = 0x92FD,    // Settings:Configuration:TS1 Config
    TS2_CONFIG                  = 0x92FE,    // Settings:Configuration:TS2 Config
    TS3_CONFIG                  = 0x92FF,    // Settings:Configuration:TS3 Config
    HDQ_PIN_CONFIG              = 0x9300,    // Settings:Configuration:HDQ Pin Config
    DCHG_PIN_CONFIG             = 0x9301,    // Settings:Configuration:DCHG Pin Config
    DDSG_PIN_CONFIG             = 0x9302,    // Settings:Configuration:DDSG Pin Config
    DA_CONFIGURATION            = 0x9303,    // Settings:Configuration:DA Configuration
    VCELL_MODE                  = 0x9304,    // Settings:Configuration:Vcell Mode
    CC3_SAMPLES                 = 0x9307,    // Settings:Configuration:CC3 Samples
    PROTECTION_CONFIGURATION    = 0x925F,    // Settings:Protection:Protection Configuration
    ENABLED_PROTECTIONS_A       = 0x9261,    // Settings:Protection:Enabled Protections A
    ENABLED_PROTECTIONS_B       = 0x9262,    // Settings:Protection:Enabled Protections B
    ENABLED_PROTECTIONS_C       = 0x9263,    // Settings:Protection:Enabled Protections C
    CHG_FET_PROTECTIONS_A       = 0x9265,    // Settings:Protection:CHG FET Protections A
    CHG_FET_PROTECTIONS_B       = 0x9266,    // Settings:Protection:CHG FET Protections B
    CHG_FET_PROTECTIONS_C       = 0x9267,    // Settings:Protection:CHG FET Protections C
    DSG_FET_PROTECTIONS_A       = 0x9269,    // Settings:Protection:DSG FET Protections A
    DSG_FET_PROTECTIONS_B       = 0x926A,    // Settings:Protection:DSG FET Protections B
    DSG_FET_PROTECTIONS_C       = 0x926B,    // Settings:Protection:DSG FET Protections C
    BODY_DIODE_THRESHOLD        = 0x9273,    // Settings:Protection:Body Diode Threshold
    DEFAULT_ALARM_MASK          = 0x926D,    // Settings:Alarm:Default Alarm Mask
    SF_ALERT_MASK_A             = 0x926F,    // Settings:Alarm:SF Alert Mask A
    SF_ALERT_MASK_B             = 0x9270,    // Settings:Alarm:SF Alert Mask B
    SF_ALERT_MASK_C             = 0x9271,    // Settings:Alarm:SF Alert Mask C
    PF_ALERT_MASK_A             = 0x92C4,    // Settings:Alarm:PF Alert Mask A
    PF_ALERT_MASK_B             = 0x92C5,    // Settings:Alarm:PF Alert Mask B
    PF_ALERT_MASK_C             = 0x92C6,    // Settings:Alarm:PF Alert Mask C
    PF_ALERT_MASK_D             = 0x92C7,    // Settings:Alarm:PF Alert Mask D
    ENABLED_PF_A                = 0x92C0,    // Settings:Permanent Failure:Enabled PF A
    ENABLED_PF_B                = 0x92C1,    // Settings:Permanent Failure:Enabled PF B
    ENABLED_PF_C                = 0x92C2,    // Settings:Permanent Failure:Enabled PF C
    ENABLED_PF_D                = 0x92C3,    // Settings:Permanent Failure:Enabled PF D
    FET_OPTIONS                 = 0x9308,    // Settings:FET:FET Options
    CHG_PUMP_CONTROL            = 0x9309,    // Settings:FET:Chg Pump Control
    PRECHARGE_START_VOLTAGE     = 0x930A,    // Settings:FET:Precharge Start Voltage
    PRECHARGE_STOP_VOLTAGE      = 0x930C,    // Settings:FET:Precharge Stop Voltage
    PREDISCHARGE_TIMEOUT        = 0x930E,    // Settings:FET:Predischarge Timeout
    PREDISCHARGE_STOP_DELTA     = 0x930F,    // Settings:FET:Predischarge Stop Delta
    DSG_CURRENT_THRESHOLD       = 0x9310,    // Settings:Current Thresholds:Dsg Current Threshold
    CHG_CURRENT_THRESHOLD       = 0x9312,    // Settings:Current Thresholds:Chg Current Threshold
    CHECK_TIME                  = 0x9314,    // Settings:Cell Open-Wire:Check Time
    CELL1_INTERCONNECT          = 0x9315,    // Settings:Interconnect Resistances:Cell 1 Interconnect
    CELL2_INTERCONNECT          = 0x9317,    // Settings:Interconnect Resistances:Cell 2 Interconnect
    CELL3_INTERCONNECT          = 0x9319,    // Settings:Interconnect Resistances:Cell 3 Interconnect
    CELL4_INTERCONNECT          = 0x931B,    // Settings:Interconnect Resistances:Cell 4 Interconnect
    CELL5_INTERCONNECT          = 0x931D,    // Settings:Interconnect Resistances:Cell 5 Interconnect
    CELL6_INTERCONNECT          = 0x931F,    // Settings:Interconnect Resistances:Cell 6 Interconnect
    CELL7_INTERCONNECT          = 0x9321,    // Settings:Interconnect Resistances:Cell 7 Interconnect
    CELL8_INTERCONNECT          = 0x9323,    // Settings:Interconnect Resistances:Cell 8 Interconnect
    CELL9_INTERCONNECT          = 0x9325,    // Settings:Interconnect Resistances:Cell 9 Interconnect
    CELL10_INTERCONNECT         = 0x9327,    // Settings:Interconnect Resistances:Cell 10 Interconnect
    CELL11_INTERCONNECT         = 0x9329,    // Settings:Interconnect Resistances:Cell 11 Interconnect
    CELL12_INTERCONNECT         = 0x932B,    // Settings:Interconnect Resistances:Cell 12 Interconnect
    CELL13_INTERCONNECT         = 0x932D,    // Settings:Interconnect Resistances:Cell 13 Interconnect
    CELL14_INTERCONNECT         = 0x932F,    // Settings:Interconnect Resistances:Cell 14 Interconnect
    CELL15_INTERCONNECT         = 0x9331,    // Settings:Interconnect Resistances:Cell 15 Interconnect
    CELL16_INTERCONNECT         = 0x9333,    // Settings:Interconnect Resistances:Cell 16 Interconnect
    MFG_STATUS_INIT             = 0x9343,    // Settings:Manufacturing:Mfg Status Init
    BALANCING_CONFIGURATION     = 0x9335,    // Settings:Cell Balancing Config:Balancing Configuration
    MIN_CELL_TEMP               = 0x9336,    // Settings:Cell Balancing Config:Min Cell Temp
    MAX_CELL_TEMP               = 0x9337,    // Settings:Cell Balancing Config:Max Cell Temp
    MAX_INTERNAL_TEMP           = 0x9338,    // Settings:Cell Balancing Config:Max Internal Temp
    CELL_BALANCE_INTERVAL       = 0x9339,    // Settings:Cell Balancing Config:Cell Balance Interval
    CELL_BALANCE_MAX_CELLS      = 0x933A,    // Settings:Cell Balancing Config:Cell Balance Max Cells
    CELL_BALANCE_MIN_CELL_V_CHG = 0x933B,    // Settings:Cell Balancing Config:Cell Balance Min Cell V (Charge)
    CELL_BALANCE_MIN_DELTA_CHG  = 0x933D,    // Settings:Cell Balancing Config:Cell Balance Min Delta (Charge)
    CELL_BALANCE_STOP_DELTA_CHG = 0x933E,    // Settings:Cell Balancing Config:Cell Balance Stop Delta (Charge)
    CELL_BALANCE_MIN_CELL_V_REL = 0x933F,    // Settings:Cell Balancing Config:Cell Balance Min Cell V (Relax)
    CELL_BALANCE_MIN_DELTA_REL  = 0x9341,    // Settings:Cell Balancing Config:Cell Balance Min Delta (Relax)
    CELL_BALANCE_STOP_DELTA_REL = 0x9342,    // Settings:Cell Balancing Config:Cell Balance Stop Delta (Relax)
    SHUTDOWN_CELL_VOLTAGE       = 0x923F,    // Power:Shutdown:Shutdown Cell Voltage
    SHUTDOWN_STACK_VOLTAGE      = 0x9241,    // Power:Shutdown:Shutdown Stack Voltage
    LOW_V_SHUTDOWN_DELAY        = 0x9243,    // Power:Shutdown:Low V Shutdown Delay
    SHUTDOWN_TEMPERATURE        = 0x9244,    // Power:Shutdown:Shutdown Temperature
    SHUTDOWN_TEMPERATURE_DELAY  = 0x9245,    // Power:Shutdown:Shutdown Temperature Delay
    FET_OFF_DELAY               = 0x9252,    // Power:Shutdown:FET Off Delay
    SHUTDOWN_COMMAND_DELAY      = 0x9253,    // Power:Shutdown:Shutdown Command Delay
    AUTO_SHUTDOWN_TIME          = 0x9254,    // Power:Shutdown:Auto Shutdown Time
    RAM_FAIL_SHUTDOWN_TIME      = 0x9255,    // Power:Shutdown:RAM Fail Shutdown Time
    SLEEP_CURRENT               = 0x9248,    // Power:Sleep:Sleep Current
    VOLTAGE_TIME                = 0x924A,    // Power:Sleep:Voltage Time
    WAKE_COMPARATOR_CURRENT     = 0x924B,    // Power:Sleep:Wake Comparator Current
    SLEEP_HYSTERESIS_TIME       = 0x924D,    // Power:Sleep:Sleep Hysteresis Time
    SLEEP_CHG_V_THRESHOLD       = 0x924E,    // Power:Sleep:Sleep Charger Voltage Threshold
    SLEEP_CHG_PACK_TOS_DELTA    = 0x9250,    // Power:Sleep:Sleep Charger PACK-TOS Delta
    CONFIG_RAM_SIGNATURE        = 0x91E0,    // System Data:Integrity:Config RAM Signature
    CUV_THRESHOLD               = 0x9275,    // Protections:CUV:Threshold
    CUV_DELAY                   = 0x9276,    // Protections:CUV:Delay
    CUV_RECOVERY_HYSTERESIS     = 0x927B,    // Protections:CUV:Recovery Hysteresis
    COV_THRESHOLD               = 0x9278,    // Protections:COV:Threshold
    COV_DELAY                   = 0x9279,    // Protections:COV:Delay
    COV_RECOVERY_HYSTERESIS     = 0x927C,    // Protections:COV:Recovery Hysteresis
    COVL_LATCH_LIMIT            = 0x927D,    // Protections:COVL:Latch Limit
    COVL_COUNTER_DEC_DELAY      = 0x927E,    // Protections:COVL:Counter Dec Delay
    COVL_RECOVERY_TIME          = 0x927F,    // Protections:COVL:Recovery Time
    OCC_THRESHOLD               = 0x9280,    // Protections:OCC:Threshold
    OCC_DELAY                   = 0x9281,    // Protections:OCC:Delay
    OCC_RECOVERY_THRESHOLD      = 0x9288,    // Protections:OCC:Recovery Threshold
    OCC_PACK_TOS_DELTA          = 0x92B0,    // Protections:OCC:PACK-TOS Delta
    OCD1_THRESHOLD              = 0x9282,    // Protections:OCD1:Threshold
    OCD1_DELAY                  = 0x9283,    // Protections:OCD1:Delay
    OCD2_THRESHOLD              = 0x9284,    // Protections:OCD2:Threshold
    OCD2_DELAY                  = 0x9285,    // Protections:OCD2:Delay
    SCD_THRESHOLD               = 0x9286,    // Protections:SCD:Threshold
    SCD_DELAY                   = 0x9287,    // Protections:SCD:Delay
    SCD_RECOVERY_TIME           = 0x9294,    // Protections:SCD:Recovery Time
    OCD3_THRESHOLD              = 0x928A,    // Protections:OCD3:Threshold
    OCD3_DELAY                  = 0x928C,    // Protections:OCD3:Delay
    OCD_RECOVERY_THRESHOLD      = 0x928D,    // Protections:OCD:Recovery Threshold
    OCDL_LATCH_LIMIT            = 0x928F,    // Protections:OCDL:Latch Limit
    OCDL_COUNTER_DEC_DELAY      = 0x9290,    // Protections:OCDL:Counter Dec Delay
    OCDL_RECOVERY_TIME          = 0x9291,    // Protections:OCDL:Recovery Time
    OCDL_RECOVERY_THRESHOLD     = 0x9292,    // Protections:OCDL:Recovery Threshold
    SCDL_LATCH_LIMIT            = 0x9295,    // Protections:SCDL:Latch Limit
    SCDL_COUNTER_DEC_DELAY      = 0x9296,    // Protections:SCDL:Counter Dec Delay
    SCDL_RECOVERY_TIME          = 0x9297,    // Protections:SCDL:Recovery Time
    SCDL_RECOVERY_THRESHOLD     = 0x9298,    // Protections:SCDL:Recovery Threshold
    OTC_THRESHOLD               = 0x929A,    // Protections:OTC:Threshold
    OTC_DELAY                   = 0x920B,    // Protections:OTC:Delay
    OTC_RECOVERY                = 0x929C,    // Protections:OTC:Recovery
    OTD_THRESHOLD               = 0x929D,    // Protections:OTD:Threshold
    OTD_DELAY                   = 0x929E,    // Protections:OTD:Delay
    OTD_RECOVERY                = 0x929F,    // Protections:OTD:Recovery
    OTF_THRESHOLD               = 0x92A0,    // Protections:OTF:Threshold
    OTF_DELAY                   = 0x92A1,    // Protections:OTF:Delay
    OTF_RECOVERY                = 0x92A2,    // Protections:OTF:Recovery
    OTINT_THRESHOLD             = 0x92A3,    // Protections:OTINT:Threshold
    OTINT_DELAY                 = 0x92A4,    // Protections:OTINT:Delay
    OTINT_RECOVERY              = 0x92A5,    // Protections:OTINT:Recovery
    UTC_THRESHOLD               = 0x92A6,    // Protections:UTC:Threshold
    UTC_DELAY                   = 0x92A7,    // Protections:UTC:Delay
    UTC_RECOVERY                = 0x92A8,    // Protections:UTC:Recovery
    UTD_THRESHOLD               = 0x92A9,    // Protections:UTD:Threshold
    UTD_DELAY                   = 0x92AA,    // Protections:UTD:Delay
    UTD_RECOVERY                = 0x92AB,    // Protections:UTD:Recovery
    UTINT_THRESHOLD             = 0x92AC,    // Protections:UTINT:Threshold
    UTINT_DELAY                 = 0x92AD,    // Protections:UTINT:Delay
    UTINT_RECOVERY              = 0x92AE,    // Protections:UTINT:Recovery
    PROTECTIONS_RECOVERY_TIME   = 0x92AF,    // Protections:Recovery:Time
    HWD_DELAY                   = 0x92B2,    // Protections:HWD:Delay
    LOAD_DETECT_ACTIVE_TIME     = 0x92B4,    // Protections:Load Detect:Active Time
    LOAD_DETECT_RETRY_DELAY     = 0x92B5,    // Protections:Load Detect:Retry Delay
    LOAD_DETECT_TIMEOUT         = 0x92B6,    // Protections:Load Detect:Timeout
    PTO_CHARGE_THRESHOLD        = 0x92BA,    // Protections:PTO:Charge Threshold
    PTO_DELAY                   = 0x92BC,    // Protections:PTO:Delay
    PTO_RESET                   = 0x92BE,    // Protections:PTO:Reset
    CUDEP_THRESHOLD             = 0x92C8,    // Permanent Fail:CUDEP:Threshold
    CUDEP_DELAY                 = 0x92CA,    // Permanent Fail:CUDEP:Delay
    SUV_THRESHOLD               = 0x92CB,    // Permanent Fail:SUV:Threshold
    SUV_DELAY                   = 0x92CD,    // Permanent Fail:SUV:Delay
    SOV_THRESHOLD               = 0x92CE,    // Permanent Fail:SOV:Threshold
    SOV_DELAY                   = 0x92D0,    // Permanent Fail:SOV:Delay
    TOSS_THRESHOLD              = 0x92D1,    // Permanent Fail:TOS:Threshold
    TOSS_DELAY                  = 0x92D3,    // Permanent Fail:TOS:Delay
    SOCC_THRESHOLD              = 0x92D4,    // Permanent Fail:SOCC:Threshold
    SOCC_DELAY                  = 0x92D6,    // Permanent Fail:SOCC:Delay
    SOCD_THRESHOLD              = 0x92D7,    // Permanent Fail:SOCD:Threshold
    SOCD_DELAY                  = 0x92D9,    // Permanent Fail:SOCD:Delay
    SOT_THRESHOLD               = 0x92DA,    // Permanent Fail:SOT:Threshold
    SOT_DELAY                   = 0x92DB,    // Permanent Fail:SOT:Delay
    SOTF_THRESHOLD              = 0x92DC,    // Permanent Fail:SOTF:Threshold
    SOTF_DELAY                  = 0x92DD,    // Permanent Fail:SOTF:Delay
    VIMR_CHECK_VOLTAGE          = 0x92DE,    // Permanent Fail:VIMR:Check Voltage
    VIMR_MAX_RELAX_CURRENT      = 0x92E0,    // Permanent Fail:VIMR:Max Relax Current
    VIMR_THRESHOLD              = 0x92E2,    // Permanent Fail:VIMR:Threshold
    VIMR_DELAY                  = 0x92E4,    // Permanent Fail:VIMR:Delay
    VIMR_RELAX_MIN_DURATION     = 0x92E5,    // Permanent Fail:VIMR:Relax Min Duration
    VIMA_CHECK_VOLTAGE          = 0x92E7,    // Permanent Fail:VIMA:Check Voltage
    VIMA_MIN_ACTIVE_CURRENT     = 0x92E9,    // Permanent Fail:VIMA:Min Active Current
    VIMA_THRESHOLD              = 0x92EB,    // Permanent Fail:VIMA:Threshold
    VIMA_DELAY                  = 0x92ED,    // Permanent Fail:VIMA:Delay
    CFETF_OFF_THRESHOLD         = 0x92EE,    // Permanent Fail:CFETF:OFF Threshold
    CFETF_OFF_DELAY             = 0x92F0,    // Permanent Fail:CFETF:OFF Delay
    DFETF_OFF_THRESHOLD         = 0x92F1,    // Permanent Fail:DFETF:OFF Threshold
    DFETF_OFF_DELAY             = 0x92F3,    // Permanent Fail:DFETF:OFF Delay
    VSSF_FAIL_THRESHOLD         = 0x92F4,    // Permanent Fail:VSSF:Fail Threshold
    VSSF_DELAY                  = 0x92F6,    // Permanent Fail:VSSF:Delay
    PF_2LVL_DELAY               = 0x92F7,    // Permanent Fail:2LVL:Delay
    LFOF_DELAY                  = 0x92F8,    // Permanent Fail:LFOF:Delay
    HWMX_DELAY                  = 0x92F9,    // Permanent Fail:HWMX:Delay
    SECURITY_SETTINGS           = 0x9256,    // Security:Settings:Security Settings
    UNSEAL_KEY_STEP1            = 0x9257,    // Security:Keys:Unseal Key Step 1
    UNSEAL_KEY_STEP2            = 0x9259,    // Security:Keys:Unseal Key Step 2
    FULL_ACCESS_KEY_STEP1       = 0x925B,    // Security:Keys:Full Access Key Step 1
    FULL_ACCESS_KEY_STEP2       = 0x925D     // Security:Keys:Full Access Key Step 2
} data_memory_registers_t;

/* Approximate time of completion in usec */
typedef enum
{
    TIME_CONTROL_STATUS       = 50,
    TIME_SAFETY_ALERT         = 50,
    TIME_PF_ALERT             = 50,
    TIME_BATTERY_STATUS       = 50,
    TIME_CELL_VOLTAGES        = 50,
    TIME_STACK_VOLTAGE        = 50,
    TIME_PACK_PIN_VOLTAGE     = 50,
    TIME_LD_PIN_VOLTAGE       = 50,
    TIME_CC2_CURRENT          = 50,
    TIME_ALARM_STATUS         = 50,
    TIME_ALARM_RAW_STATUS     = 50,
    TIME_ALARM_ENABLE         = 50,
    TIME_INTERNAL_TEMP        = 50,
    TIME_THERMISTOR_TEMP      = 50,
    TIME_DEVICE_NUMBER        = 400,
    TIME_FW_VERSION           = 400,
    TIME_HW_VERSION           = 400,
    TIME_IROM_SIG             = 8500,
    TIME_STATIC_CFG_SIG       = 450,
    TIME_DROM_SIG             = 650,
    TIME_EXIT_DEEPSLEEP       = 500,
    TIME_DEEPSLEEP            = 500,
    TIME_SHUTDOWN             = 500,
    TIME_PDSGTEST             = 550,
    TIME_FUSE_TOGGLE          = 500,
    TIME_PCHGTEST             = 900,
    TIME_CHGTEST              = 550,
    TIME_DSGTEST              = 550,
    TIME_FET_ENABLE           = 500,
    TIME_PF_ENABLE            = 500,
    TIME_SEAL                 = 500,
    TIME_SAVED_PF_STATUS      = 500,
    TIME_MANUFACTURING_STATUS = 605,
    TIME_MANU_DATA            = 660,
    TIME_DATASTAT1            = 660,
    TIME_CUV_SNAPSHOT         = 660,
    TIME_COV_SNAPSHOT         = 660,
    TIME_RESET_PASSQ          = 600,
    TIME_CB_ACTIVE_CELLS      = 560,
    TIME_CB_SET_LVL1          = 480,
    TIME_CBSTATUS1            = 575,
    TIME_PTO_RECOVER          = 500,
    TIME_SET_CFGUPDATE        = 2000,
    TIME_EXIT_CFGUPDATE       = 1000,
    TIME_DSG_PDSG_OFF         = 550,
    TIME_CHG_PCHG_OFF         = 550,
    TIME_ALL_FETS_OFF         = 550,
    TIME_ALL_FETS_ON          = 500,
    TIME_FET_CONTROL          = 495,
    TIME_REG12_CONTROL        = 450,
    TIME_SLEEP_ENABLE         = 500,
    TIME_SLEEP_DISABLE        = 500,
    TIME_OCDL_RECOVER         = 500,
    TIME_SCDL_RECOVER         = 500,
    TIME_LOAD_DET_RESTART     = 500,
    TIME_LOAD_DET_ON          = 500,
    TIME_LOAD_DET_OFF         = 500,
    TIME_OTP_WR_CHECK         = 580,
    TIME_GPO_SUBCOMMANDS      = 500,
    TIME_PF_FORCE_A           = 500,
    TIME_PF_FORCE_B           = 800,
    TIME_SWAP_COMM_MODE       = 500,
    TIME_SWAP_TO_I2C          = 500,
    TIME_SWAP_TO_SPI          = 500,
    TIME_SWAP_TO_HDQ          = 500,
    TIME_READ_CAL1            = 630
} bms_command_time_t;

typedef enum
{
    BMS_OTP_OK = 0,
    BMS_OTP_NOT_PROGRAMMED,
    BMS_OTP_BQ_NOT_DETECTED,
    BMS_OTP_CONDITIONS_NOT_MET,
    BMS_OTP_FAILED,
    BMS_OTP_ERROR
} bms_otp_status_t;

typedef enum
{
    BMS_STATE_INACTIVE = 0,
    BMS_STATE_ACTIVE,
    BMS_STATE_SLEEP,
    BMS_STATE_FAULT,
    BMS_STATE_UNSPECIFIED
} bms_state_t;

typedef enum
{
    BMS_CMD_NONE = 0
} bms_command_t;

/* Exported constants and defines --------------------------------------------*/
/** @brief BMS Model Selection */
#if defined(BMS_MODEL_10S)
    #define BMS_CELL_COUNT            (10u)
    #define BMS_CONSECUTIVE_CELLS     (9u)
#elif defined(BMS_MODEL_14S)
    #define BMS_CELL_COUNT            (14u)
    #define BMS_CONSECUTIVE_CELLS     (13u)
#else
    #error "BMS model not defined. Define BMS_MODEL_10S or BMS_MODEL_14S"
#endif

#define BMS_LAST_CELL_INDEX       (15u)    /* Cell 16 address index (0-based) */


/* Exported macros -----------------------------------------------------------*/


/* Exported function prototypes ----------------------------------------------*/
void bms_init(void);
void bms_dfet_off(void);
void bms_reset_shutdown(void);

void CommandSubcommands(uint16_t command);
void Subcommands(uint16_t command, uint16_t data, uint8_t type);
void DirectCommands(uint8_t command, uint16_t data, uint8_t type);

void BQ769x2_ReadPFStatus(void);
void BQ769x2_ReadFETStatus(void);
void BQ769x2_ReadSafetyStatus(void);
void BQ769x2_Readcell_voltages(void);
void BQ769x2_SetRegister(uint16_t reg_addr, uint32_t reg_data, uint8_t datalen);

uint8_t bms_get_faults(void);
uint8_t get_fet_status(void);
uint8_t get_bms_status(void);
uint8_t get_charging_status(void);
uint8_t get_balancing_status(void);

int16_t BQ769x2_ReadCurrent(void);

uint16_t BQ769x2_ReadAlarmStatus(void);
uint16_t get_largest_cell_voltage(void);
uint16_t get_smallest_cell_voltage(void);
uint16_t BQ769x2_ReadVoltage(uint8_t command);

float BQ769x2_ReadTemperature(uint8_t command);

bms_otp_status_t bms_otp_check(void);

void read_cuv_voltages(void);
void BQ769x2_readall_voltages(void);

#endif /* INC_BMS_H_ */
