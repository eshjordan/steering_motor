#pragma once

#include <stdint.h>

typedef struct __attribute__((__packed__)) {
    const uint8_t sof : 1 = 0b0;
    uint16_t id : 11;
    uint8_t stuff : 1;
    uint8_t rtr : 1;
    const uint8_t ide : 1      = 0b0;
    const uint8_t reserved : 1 = 0b0;
    uint8_t dlc : 4;
    uint8_t data_00 : 8;
    uint8_t data_01 : 8;
    uint8_t data_02 : 8;
    uint8_t data_03 : 8;
    uint8_t data_04 : 8;
    uint8_t data_05 : 8;
    uint8_t data_06 : 8;
    uint8_t data_07 : 8;
    uint16_t crc : 15;
    const uint8_t crc_delimiter : 1 = 0b1;
    uint8_t ack_slot : 1;
    const uint8_t acl_delimiter : 1 = 0b1;
    const uint8_t eof : 7           = 0b1111111;
    const uint8_t ifs : 3           = 0b111;
} can_raw_frame_t;

/**
 * @brief CANopen function specifier.
 *
 */
typedef enum : uint16_t {
    CANOPEN_FN_NMT         = 0x000U, // NMT node control, slave RX
    CANOPEN_FN_FAILSAFE    = 0x001U, // Global failsafe command
    CANOPEN_FN_SYNC        = 0x080U, // Sync (see heartbeat/Timer), slave RX
    CANOPEN_FN_EMERGENCY   = 0x080U, // Emergency, + NodeID, slave TX
    CANOPEN_FN_TIMESTAMP   = 0x100U, // Timestamp, slave RX
    CANOPEN_FN_PDO_TX_01   = 0x180U, // PDO 01 slave TX, + NodeID
    CANOPEN_FN_PDO_RX_01   = 0x200U, // PDO 01 slave RX, + NodeID
    CANOPEN_FN_PDO_TX_02   = 0x280U, // PDO 02 slave TX, + NodeID
    CANOPEN_FN_PDO_RX_02   = 0x300U, // PDO 02 slave RX, + NodeID
    CANOPEN_FN_PDO_TX_03   = 0x380U, // PDO 03 slave TX, + NodeID
    CANOPEN_FN_PDO_RX_03   = 0x400U, // PDO 03 slave RX, + NodeID
    CANOPEN_FN_PDO_TX_04   = 0x480U, // PDO 04 slave TX, + NodeID
    CANOPEN_FN_PDO_RX_04   = 0x500U, // PDO 04 slave RX, + NodeID
    CANOPEN_FN_SDO_TX      = 0x580U, // SDO slave TX, (works in pre-operational State), + NodeID
    CANOPEN_FN_SDO_RX      = 0x600U, // SDO slave RX, (works in pre-operational State), + NodeID
    CANOPEN_FN_NMT_MONITOR = 0x700U, // NMT node monitoring, slave TX, + NodeID
    CANOPEN_FN_LSS_TX      = 0x7E4U, // Layer Setting Service (LSS), slave TX
    CANOPEN_FN_LSS_RX      = 0x7E5U, // Layer Setting Service (LSS), slave RX
} CANopenFunction_en;

/**
 * @brief NMT command code
 *
 */
typedef enum : uint8_t {
    NMT_COMMAND_OPERATIONAL          = 0x01, // Go to 'operational'
    NMT_COMMAND_STOPPED              = 0x02, // Go to 'stopped'
    NMT_COMMAND_PRE_OPERATIONAL      = 0x80, // Go to 'pre-operational'
    NMT_COMMAND_RESET_NODE           = 0x81, // Go to 'reset node'
    NMT_COMMAND_RESET_COMMUNNICATION = 0x82, // Go to 'reset communication'
} NMT_Command_en;

/**
 * @brief NMT is the Network Management protocol.
 *
 */
typedef struct {
    NMT_Command_en requested_state;
    uint8_t addressed_node; // Node to set state of, if 0, send to all nodes on the bus
} NMT_t;

typedef enum : uint8_t {
    NMT_STATE_BOOTUP          = 0x00U,
    NMT_STATE_STOPPED         = 0x04U,
    NMT_STATE_OPERATIONAL     = 0x05U,
    NMT_STATE_PRE_OPERATIONAL = 0x7fU,
} NMT_State_en;

typedef struct {
    NMT_State_en state;
} NMT_Heartbeat_t;

/**
 * @brief CCS is the client command specifier of the SDO transfer.
 *
 */
typedef enum : uint8_t {
    CCS_DOWNLOAD          = 0x00U, // SDO segment download
    CCS_INITIATE_DOWNLOAD = 0x01U, // SDO initiating download
    CCS_INITIATE_UPLOAD   = 0x02U, // SDO initiating upload
    CCS_UPLOAD            = 0x03U, // SDO segment upload
    CCS_ABORT             = 0x04U, // SDO abort transfer
    CCS_BLOCK_UPLOAD      = 0x05U, // SDO block upload
    CCS_BLOCK_DOWNLOAD    = 0x06U, // SDO block download
} CCS_en;

#define SDO_EXPIDITED_DISABLE 0b0
#define SDO_EXPIDITED_ENABLE 0b1

#define SDO_SIZE_DISABLE 0b0
#define SDO_SIZE_ENABLE 0b1

/**
 * @brief SDO message structure.
 *
 */
typedef struct __attribute__((__packed__)) {
    uint8_t ccs : 3;                  // Client Command Specifier
    const uint8_t reserved_0 : 1 = 0; // Reserved
    uint8_t n : 2; // number of bytes in the data part of the message which do not contain data, only valid if e and s
                   // are set
    uint8_t e : 1; // if set, indicates an expedited transfer, i.e. all data exchanged are contained within the message.
                   // If this bit is cleared then the message is a segmented transfer where the data does not fit into
                   // one message and multiple messages are used.
    uint8_t s : 1; // if set, indicates that the data size is specified in n (if e is set) or in the data part of the
                   // message
    uint16_t index;   // object dictionary index of the data to be accessed
    uint8_t subindex; // subindex of the object dictionary variable
    uint8_t data[4];  // data to be uploaded in the case of an expedited transfer (e is set), or the size of the data to
                      // be uploaded (s is set, e is not set)
} SDO_t;

using can_rx_fn = uint8_t (*)(struct can_frame *frame);

using can_tx_fn = uint8_t (*)(const struct can_frame *frame);

/** @brief Communication Profile Objects **/
constexpr uint16_t Device_Type                             = 0x1000U;
constexpr uint16_t Error_Register                          = 0x1001U;
constexpr uint16_t COB_ID_SYNC                             = 0x1005U;
constexpr uint16_t Communication_Cycle_Period              = 0x1006U;
constexpr uint16_t Manufacturer_Device_Name                = 0x1008U;
constexpr uint16_t Manufacturer_Hardware_Version           = 0x1009U;
constexpr uint16_t Manufacturer_Software_Version           = 0x100AU;
constexpr uint16_t High_Resolution_Timestamp               = 0x1013U;
constexpr uint16_t Producer_Heartbeat_Time                 = 0x1017U;
constexpr uint16_t Identity_Object                         = 0x1018U;
constexpr uint16_t Server_SDO_Parameter_01                 = 0x1200U;
constexpr uint16_t Receive_PDO_Communication_Parameter_01  = 0x1400U;
constexpr uint16_t Receive_PDO_Communication_Parameter_02  = 0x1401U;
constexpr uint16_t Receive_PDO_Communication_Parameter_03  = 0x1402U;
constexpr uint16_t Receive_PDO_Communication_Parameter_04  = 0x1403U;
constexpr uint16_t Receive_PDO_Communication_Parameter_05  = 0x1404U;
constexpr uint16_t Receive_PDO_Mapping_Parameter_01        = 0x1600U;
constexpr uint16_t Receive_PDO_Mapping_Parameter_02        = 0x1601U;
constexpr uint16_t Receive_PDO_Mapping_Parameter_03        = 0x1602U;
constexpr uint16_t Receive_PDO_Mapping_Parameter_04        = 0x1603U;
constexpr uint16_t Receive_PDO_Mapping_Parameter_05        = 0x1604U;
constexpr uint16_t Transmit_PDO_Communication_Parameter_01 = 0x1800U;
constexpr uint16_t Transmit_PDO_Communication_Parameter_02 = 0x1801U;
constexpr uint16_t Transmit_PDO_Communication_Parameter_03 = 0x1802U;
constexpr uint16_t Transmit_PDO_Communication_Parameter_04 = 0x1803U;
constexpr uint16_t Transmit_PDO_Communication_Parameter_05 = 0x1804U;
constexpr uint16_t Transmit_PDO_Mapping_Parameter_01       = 0x1A00U;
constexpr uint16_t Transmit_PDO_Mapping_Parameter_02       = 0x1A01U;
constexpr uint16_t Transmit_PDO_Mapping_Parameter_03       = 0x1A02U;
constexpr uint16_t Transmit_PDO_Mapping_Parameter_04       = 0x1A03U;
constexpr uint16_t Transmit_PDO_Mapping_Parameter_05       = 0x1A04U;

/** @brief Manufactuer Specific Objects **/
constexpr uint16_t Node_Id                          = 0x2000U;
constexpr uint16_t Bit_Rate_Index                   = 0x2001U;
constexpr uint16_t Port_Configuration               = 0x2100U;
constexpr uint16_t Bit_IO                           = 0x2101U;
constexpr uint16_t User_EEPROM                      = 0x2200U;
constexpr uint16_t User_Variable                    = 0x2201U;
constexpr uint16_t Set_Position_Origin              = 0x2202U;
constexpr uint16_t Shift_Position_Origin            = 0x2203U;
constexpr uint16_t Mappable_32_bit_Variables        = 0x2204U;
constexpr uint16_t Negative_Software_Position_Limit = 0x2205U;
constexpr uint16_t Positive_Software_Position_Limit = 0x2206U;
constexpr uint16_t Encoder_Modulo_Limit             = 0x2207U;
constexpr uint16_t Encoder_Follow_Data              = 0x2208U;
constexpr uint16_t Encoder_Follow_Control           = 0x2209U;
constexpr uint16_t MFMUL                            = 0x220AU;
constexpr uint16_t MFDIV                            = 0x220BU;
constexpr uint16_t MFA                              = 0x220CU;
constexpr uint16_t MFD                              = 0x220DU;
constexpr uint16_t Eight_Bit_Mappable_Variables     = 0x2220U;
constexpr uint16_t Sixteen_Bit_Mappable_Variables   = 0x2221U;
constexpr uint16_t Bus_Voltage                      = 0x2300U;
constexpr uint16_t RMS_Current                      = 0x2301U;
constexpr uint16_t Internal_Temperature             = 0x2302U;
constexpr uint16_t Internal_Clock                   = 0x2303U;
constexpr uint16_t Motor_Status                     = 0x2304U;
constexpr uint16_t Motor_Control                    = 0x2305U;
constexpr uint16_t Motor_Subroutine_Index           = 0x2306U;
constexpr uint16_t Sample_Period                    = 0x2307U;
constexpr uint16_t Microsecond_Clock                = 0x2308U;
constexpr uint16_t GOSUB_R2                         = 0x2309U;
constexpr uint16_t Interpolation_Mode_Status        = 0x2400U;
constexpr uint16_t Buffer_Control                   = 0x2401U;
constexpr uint16_t Buffer_Setpoint                  = 0x2402U;
constexpr uint16_t Interpolation_User_Bits          = 0x2403U;
constexpr uint16_t Interpolation_Sample_Clock       = 0x2404U;
constexpr uint16_t Encapsulated_SmartMotor_Command  = 0x2500U;

/** @brief Drive and Motion Control Objects **/
constexpr uint16_t Control_Word                     = 0x6040U;
constexpr uint16_t Status_Word                      = 0x6041U;
constexpr uint16_t Quick_Stop_Option_Code           = 0x605AU;
constexpr uint16_t Halt_Option_Code                 = 0x605DU;
constexpr uint16_t Fault_Reaction_Option_Code       = 0x605EU;
constexpr uint16_t Modes_of_Operation               = 0x6060U;
constexpr uint16_t Modes_of_Operation_Display       = 0x6061U;
constexpr uint16_t Position_Demand_Value            = 0x6062U;
constexpr uint16_t Position_Actual_Internal_Value   = 0x6063U;
constexpr uint16_t Position_Actual_Value            = 0x6064U;
constexpr uint16_t Following_Error_Window           = 0x6065U;
constexpr uint16_t Velocity_Demand_Value            = 0x606BU;
constexpr uint16_t Velocity_Actual_Value            = 0x606CU;
constexpr uint16_t Target_Torque                    = 0x6071U;
constexpr uint16_t Torque_Demand_Value              = 0x6074U;
constexpr uint16_t Torque_Actual                    = 0x6077U;
constexpr uint16_t DC_Link_Circuit_Voltage          = 0x6079U;
constexpr uint16_t Target_Position                  = 0x607AU;
constexpr uint16_t Home_Offset                      = 0x607CU;
constexpr uint16_t Max_Motor_Speed                  = 0x6080U;
constexpr uint16_t Profile_Velocity_in_PP_Mode      = 0x6081U;
constexpr uint16_t Profile_Acceleration             = 0x6083U;
constexpr uint16_t Profile_Deceleration             = 0x6084U;
constexpr uint16_t Quick_Stop_Deceleration          = 0x6085U;
constexpr uint16_t Torque_Slope                     = 0x6087U;
constexpr uint16_t Position_Encoder_Resolution      = 0x608FU;
constexpr uint16_t Homing_Method                    = 0x6098U;
constexpr uint16_t Homing_Speeds                    = 0x6099U;
constexpr uint16_t Homing_Acceleration              = 0x609AU;
constexpr uint16_t Interpolation_Sub_Mode_Select    = 0x60C0U;
constexpr uint16_t Interpolation_Data_Record        = 0x60C1U;
constexpr uint16_t Interpolation_Time_Period        = 0x60C2U;
constexpr uint16_t Interpolation_Data_Configuration = 0x60C4U;
constexpr uint16_t Following_Error_Actual_Value     = 0x60F4U;
constexpr uint16_t Position_Control_Parameter_Set   = 0x60FBU;
constexpr uint16_t Position_Demand_Internal_Value   = 0x60FCU;
constexpr uint16_t Digital_Inputs                   = 0x60FDU;
constexpr uint16_t Digital_Outputs                  = 0x60FEU;
constexpr uint16_t Target_Velocity                  = 0x60FFU;
constexpr uint16_t Motor_Type                       = 0x6402U;
constexpr uint16_t Supported_Drive_Modes            = 0x6502U;
constexpr uint16_t Single_Device_Type               = 0x67FFU;
