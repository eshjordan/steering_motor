#pragma once

#include <stdint.h>
#include <stdio.h>

#define TIME_OUT 10000

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)                                                                                           \
    (byte & 0x80U ? '1' : '0'), (byte & 0x40U ? '1' : '0'), (byte & 0x20U ? '1' : '0'), (byte & 0x10U ? '1' : '0'),    \
        (byte & 0x08U ? '1' : '0'), (byte & 0x04U ? '1' : '0'), (byte & 0x02U ? '1' : '0'), (byte & 0x01U ? '1' : '0')

#define SET_BIT(value, bit) ((value) |= (__typeof__(value))(1u << (uint8_t)(bit)))
#define CLEAR_BIT(value, bit) ((value) &= (__typeof__(value))(~(__typeof__(value))(1u << (uint8_t)(bit))))
#define SET_BIT_PATTERN(value, pattern) ((value) |= (__typeof__(value))(pattern))
#define CLEAR_BIT_PATTERN(value, pattern) ((value) &= (__typeof__(value))(~(__typeof__(value))(pattern)))

#define SDO_EXPEDITED_DISABLE 0b0U
#define SDO_EXPEDITED_ENABLE 0b1U

#define SDO_SIZE_DISABLE 0b0U
#define SDO_SIZE_ENABLE 0b1U

/** Utility Functions **/

inline void print_byte_binary(uint8_t data) { printf(BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(data)); }

inline void print_can_frame_data(const void *data)
{
    printf("0x");
    for (uint8_t i = 0; i < 8; i++)
    {
        printf("%02hhx", ((uint8_t *)data)[i]);
    }
    printf("\n");
}

constexpr inline uint32_t radians_to_encoder_count(const double radians)
{
    constexpr uint16_t encoder_resolution = 8000U;
    constexpr double pi                   = 3.14159265358979323846;
    return (encoder_resolution * radians) / (2 * pi);
}

constexpr inline uint32_t calculate_velocity(const double rad_per_sec)
{
    constexpr uint16_t sample_rate_hz = 8000U;
    constexpr double sample_period    = 1.0 / sample_rate_hz;
    uint32_t encoder_count            = radians_to_encoder_count(rad_per_sec);
    return (encoder_count * sample_period) * (1U << 16U);
}

/** Types **/

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
    NMT_COMMAND_OPERATIONAL          = 0x01U, // Go to 'operational'
    NMT_COMMAND_STOPPED              = 0x02U, // Go to 'stopped'
    NMT_COMMAND_PRE_OPERATIONAL      = 0x80U, // Go to 'pre-operational'
    NMT_COMMAND_RESET_NODE           = 0x81U, // Go to 'reset node'
    NMT_COMMAND_RESET_COMMUNNICATION = 0x82U, // Go to 'reset communication'
} NMT_Command_en;

/**
 * @brief NMT is the Network Management protocol.
 *
 */
typedef struct {
    NMT_Command_en requested_state;
    uint8_t addressed_node; // Node to set state of, if 0, send to all nodes on the bus
} NMT_t;

/**
 * @brief NMT States
 *
 */
typedef enum : uint8_t {
    NMT_STATE_BOOTUP          = 0x00U,
    NMT_STATE_STOPPED         = 0x04U,
    NMT_STATE_OPERATIONAL     = 0x05U,
    NMT_STATE_PRE_OPERATIONAL = 0x7fU,
} NMT_State_en;

/**
 * @brief NMT Data Structure
 *
 */
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

/**
 * @brief SDO message structure.
 *
 */
typedef struct __attribute__((__packed__)) {
    uint8_t s : 1; // if set, indicates that the data size is specified in n (if e is set) or in the data part of the
                   // message
    uint8_t e : 1; // if set, indicates an expedited transfer, i.e. all data exchanged are contained within the message.
                   // If this bit is cleared then the message is a segmented transfer where the data does not fit into
                   // one message and multiple messages are used.
    uint8_t n : 2; // number of bytes in the data part of the message which do not contain data, only valid if e and s
                   // are set
    const uint8_t reserved_0 : 1; // Reserved
    CCS_en ccs : 3;               // Client Command Specifier
    uint16_t index;               // object dictionary index of the data to be accessed
    uint8_t subindex;             // subindex of the object dictionary variable
    uint8_t data[4]; // data to be uploaded in the case of an expedited transfer (e is set), or the size of the data to
                     // be uploaded (s is set, e is not set)
} SDO_t;

/**
 * @brief Status Word bit structure
 *
 */
typedef enum : uint16_t {
    SW_READY_TO_SWITCH_ON    = (1U << 0U),
    SW_SWITCHED_ON           = (1U << 1U),
    SW_OPERATION_ENABLED     = (1U << 2U),
    SW_FAULT                 = (1U << 3U),
    SW_VOLTAGE_ENABLED       = (1U << 4U),
    SW_QUICK_STOP            = (1U << 5U),
    SW_SWITCH_ON_DISABLED    = (1U << 6U),
    SW_WARNING               = (1U << 7U),
    SW_GOSUB_R2_BUSY         = (1U << 8U),
    SW_REMOTE                = (1U << 9U),
    SW_TARGET_REACHED        = (1U << 10U),
    SW_INTERNAL_LIMIT_ACTIVE = (1U << 11U),
    SW_SETPOINT_ACK          = (1U << 12U), // In Homing Mode, this is Position Actual adjusted
    SW_MOVE_ERROR            = (1U << 13U),
    SW_USER_CONTROLLED       = (1U << 14U),
    SW_UNUSED                = (1U << 15U),
    SW_MAX                   = (0xFFFFU)
} StatusWord_en;

/**
 * @brief Control Word bit structure
 *
 */
typedef enum : uint16_t {
    CW_SWITCH_ON        = (1U << 0U),
    CW_ENABLE_VOLTAGE   = (1U << 1U),
    CW_QUICK_STOP       = (1U << 2U),
    CW_ENABLE_OPERATION = (1U << 3U),
    CW_NEW_SETPOINT     = (1U << 4U),
    CW_CHANGE_SET_NOW   = (1U << 5U),
    CW_RELATIVE         = (1U << 6U),
    CW_FAULT_RESET      = (1U << 7U),
    CW_HALT             = (1U << 8U),
    CW_PP_SPECIFIC      = (1U << 9U),
    CW_RESERVED         = (1U << 10U),
    CW_USER_RCAN2       = (1U << 11U),
    CW_UNUSED_01        = (1U << 12U),
    CW_UNUSED_02        = (1U << 13U),
    CW_UNUSED_03        = (1U << 14U),
    CW_IP_BUF_RESET     = (1U << 15U),
    CW_MAX              = (0xFFFFU)
} ControlWord_en;

/**
 * @brief Motor Control Mode bytes
 *
 */
typedef enum : int8_t {
    MC_CAM_ENCODER_POSITION_MODE  = -12,
    MC_ENCODER_POSITION_MODE      = -11,
    MC_RESERVED_01_MODE           = -10,
    MC_RESERVED_02_MODE           = -9,
    MC_RESERVED_03_MODE           = -8,
    MC_RESERVED_04_MODE           = -7,
    MC_RESERVED_05_MODE           = -6,
    MC_RESERVED_06_MODE           = -5,
    MC_RESERVED_07_MODE           = -4,
    MC_STEP_DIRECTION_MODE        = -3,
    MC_FOLLOW_QUAD_ENCODER_MODE   = -2,
    MC_RESERVED_08_MODE           = -1,
    MC_NULL_MODE                  = 0,
    MC_PROFILE_POSITION_MODE      = 1,
    MC_RESERVED_09_MODE           = 2,
    MC_PROFILE_VELOCITY_MODE      = 3,
    MC_PROFILE_TORQUE_MODE        = 4,
    MC_RESERVED_10_MODE           = 5,
    MC_HOMING_MODE                = 6,
    MC_INTERPOLATED_POSITION_MODE = 7,
    MC_CYCLIC_SYNC_POSITION_MODE  = 8,
    MC_MAX                        = 9,
} MotionControlMode_en;

/**
 * @brief Generic CAN receive function type
 *
 */
typedef uint8_t (*can_rx_fn)(struct can_frame *frame, uint16_t cob_id);

/**
 * @brief Generic CAN transmit function type
 *
 */
typedef uint8_t (*can_tx_fn)(const struct can_frame *frame);

/**
 * @brief CANopen object dictionary structure for the motor.
 *
 */
typedef enum : uint16_t {
    /** @brief Communication Profile Objects **/
    OBJ_Device_Type                             = 0x1000U,
    OBJ_Error_Register                          = 0x1001U,
    OBJ_COB_ID_SYNC                             = 0x1005U,
    OBJ_Communication_Cycle_Period              = 0x1006U,
    OBJ_Manufacturer_Device_Name                = 0x1008U,
    OBJ_Manufacturer_Hardware_Version           = 0x1009U,
    OBJ_Manufacturer_Software_Version           = 0x100AU,
    OBJ_High_Resolution_Timestamp               = 0x1013U,
    OBJ_Producer_Heartbeat_Time                 = 0x1017U,
    OBJ_Identity_Object                         = 0x1018U,
    OBJ_Server_SDO_Parameter_01                 = 0x1200U,
    OBJ_Receive_PDO_Communication_Parameter_01  = 0x1400U,
    OBJ_Receive_PDO_Communication_Parameter_02  = 0x1401U,
    OBJ_Receive_PDO_Communication_Parameter_03  = 0x1402U,
    OBJ_Receive_PDO_Communication_Parameter_04  = 0x1403U,
    OBJ_Receive_PDO_Communication_Parameter_05  = 0x1404U,
    OBJ_Receive_PDO_Mapping_Parameter_01        = 0x1600U,
    OBJ_Receive_PDO_Mapping_Parameter_02        = 0x1601U,
    OBJ_Receive_PDO_Mapping_Parameter_03        = 0x1602U,
    OBJ_Receive_PDO_Mapping_Parameter_04        = 0x1603U,
    OBJ_Receive_PDO_Mapping_Parameter_05        = 0x1604U,
    OBJ_Transmit_PDO_Communication_Parameter_01 = 0x1800U,
    OBJ_Transmit_PDO_Communication_Parameter_02 = 0x1801U,
    OBJ_Transmit_PDO_Communication_Parameter_03 = 0x1802U,
    OBJ_Transmit_PDO_Communication_Parameter_04 = 0x1803U,
    OBJ_Transmit_PDO_Communication_Parameter_05 = 0x1804U,
    OBJ_Transmit_PDO_Mapping_Parameter_01       = 0x1A00U,
    OBJ_Transmit_PDO_Mapping_Parameter_02       = 0x1A01U,
    OBJ_Transmit_PDO_Mapping_Parameter_03       = 0x1A02U,
    OBJ_Transmit_PDO_Mapping_Parameter_04       = 0x1A03U,
    OBJ_Transmit_PDO_Mapping_Parameter_05       = 0x1A04U,

    /** @brief Manufactuer Specific Objects **/
    OBJ_Node_Id                          = 0x2000U,
    OBJ_Bit_Rate_Index                   = 0x2001U,
    OBJ_Port_Configuration               = 0x2100U,
    OBJ_Bit_IO                           = 0x2101U,
    OBJ_User_EEPROM                      = 0x2200U,
    OBJ_User_Variable                    = 0x2201U,
    OBJ_Set_Position_Origin              = 0x2202U,
    OBJ_Shift_Position_Origin            = 0x2203U,
    OBJ_Mappable_32_bit_Variables        = 0x2204U,
    OBJ_Negative_Software_Position_Limit = 0x2205U,
    OBJ_Positive_Software_Position_Limit = 0x2206U,
    OBJ_Encoder_Modulo_Limit             = 0x2207U,
    OBJ_Encoder_Follow_Data              = 0x2208U,
    OBJ_Encoder_Follow_Control           = 0x2209U,
    OBJ_MFMUL                            = 0x220AU,
    OBJ_MFDIV                            = 0x220BU,
    OBJ_MFA                              = 0x220CU,
    OBJ_MFD                              = 0x220DU,
    OBJ_Eight_Bit_Mappable_Variables     = 0x2220U,
    OBJ_Sixteen_Bit_Mappable_Variables   = 0x2221U,
    OBJ_Bus_Voltage                      = 0x2300U,
    OBJ_RMS_Current                      = 0x2301U,
    OBJ_Internal_Temperature             = 0x2302U,
    OBJ_Internal_Clock                   = 0x2303U,
    OBJ_Motor_Status                     = 0x2304U,
    OBJ_Motor_Control                    = 0x2305U,
    OBJ_Motor_Subroutine_Index           = 0x2306U,
    OBJ_Sample_Period                    = 0x2307U,
    OBJ_Microsecond_Clock                = 0x2308U,
    OBJ_GOSUB_R2                         = 0x2309U,
    OBJ_Interpolation_Mode_Status        = 0x2400U,
    OBJ_Buffer_Control                   = 0x2401U,
    OBJ_Buffer_Setpoint                  = 0x2402U,
    OBJ_Interpolation_User_Bits          = 0x2403U,
    OBJ_Interpolation_Sample_Clock       = 0x2404U,
    OBJ_Encapsulated_SmartMotor_Command  = 0x2500U,

    /** @brief Drive and Motion Control Objects **/
    OBJ_Control_Word                     = 0x6040U,
    OBJ_Status_Word                      = 0x6041U,
    OBJ_Quick_Stop_Option_Code           = 0x605AU,
    OBJ_Halt_Option_Code                 = 0x605DU,
    OBJ_Fault_Reaction_Option_Code       = 0x605EU,
    OBJ_Modes_of_Operation               = 0x6060U,
    OBJ_Modes_of_Operation_Display       = 0x6061U,
    OBJ_Position_Demand_Value            = 0x6062U,
    OBJ_Position_Actual_Internal_Value   = 0x6063U,
    OBJ_Position_Actual_Value            = 0x6064U,
    OBJ_Following_Error_Window           = 0x6065U,
    OBJ_Velocity_Demand_Value            = 0x606BU,
    OBJ_Velocity_Actual_Value            = 0x606CU,
    OBJ_Target_Torque                    = 0x6071U,
    OBJ_Torque_Demand_Value              = 0x6074U,
    OBJ_Torque_Actual                    = 0x6077U,
    OBJ_DC_Link_Circuit_Voltage          = 0x6079U,
    OBJ_Target_Position                  = 0x607AU,
    OBJ_Home_Offset                      = 0x607CU,
    OBJ_Max_Motor_Speed                  = 0x6080U,
    OBJ_Profile_Velocity_in_PP_Mode      = 0x6081U,
    OBJ_Profile_Acceleration             = 0x6083U,
    OBJ_Profile_Deceleration             = 0x6084U,
    OBJ_Quick_Stop_Deceleration          = 0x6085U,
    OBJ_Torque_Slope                     = 0x6087U,
    OBJ_Position_Encoder_Resolution      = 0x608FU,
    OBJ_Homing_Method                    = 0x6098U,
    OBJ_Homing_Speeds                    = 0x6099U,
    OBJ_Homing_Acceleration              = 0x609AU,
    OBJ_Interpolation_Sub_Mode_Select    = 0x60C0U,
    OBJ_Interpolation_Data_Record        = 0x60C1U,
    OBJ_Interpolation_Time_Period        = 0x60C2U,
    OBJ_Interpolation_Data_Configuration = 0x60C4U,
    OBJ_Following_Error_Actual_Value     = 0x60F4U,
    OBJ_Position_Control_Parameter_Set   = 0x60FBU,
    OBJ_Position_Demand_Internal_Value   = 0x60FCU,
    OBJ_Digital_Inputs                   = 0x60FDU,
    OBJ_Digital_Outputs                  = 0x60FEU,
    OBJ_Target_Velocity                  = 0x60FFU,
    OBJ_Motor_Type                       = 0x6402U,
    OBJ_Supported_Drive_Modes            = 0x6502U,
    OBJ_Single_Device_Type               = 0x67FFU,
} CANopenObject_en;
