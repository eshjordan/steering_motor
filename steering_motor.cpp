
#include "DriveStateController.hpp"
#if defined ENABLE_CPP_BUILD

#include "CANopen.hpp"
#include "CANopen_types.hpp"
#include "endian.hpp"
#include "unit_conversions.hpp"
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

constexpr uint8_t SM_NODE_ID = 0x4D;

uint8_t rx_fn(struct can_frame * /*frame*/, uint16_t) { return 0; }

uint8_t tx_fn(const struct can_frame *frame)
{
    printf("Sending CAN ID 0x%03lx, dlc: %hhu, data: ", frame->can_id, frame->can_dlc);
    print_can_frame_data(frame->data);
    return 0;
}

const CANopen CANOPEN(SM_NODE_ID, rx_fn, tx_fn);
DriveStateController drive_state_controller(CANOPEN, DriveStateMachine::DS_SWITCH_ON_READY);

void initMotor(uint32_t prof_vel, uint32_t prof_accel, uint32_t prof_decel)
{
    printf("Clear Faults - Expected:            0x2b40600000000000\n");
    ControlWord_en control_word = CW_FAULT_RESET;
    CANOPEN.sdo_write(OBJ_Control_Word, 0, &control_word, sizeof(control_word));

    printf("Unset Clear Faults - Expected:      0x2b40600000000000\n");
    control_word = (ControlWord_en)0U;
    CANOPEN.sdo_write(OBJ_Control_Word, 0, &control_word, sizeof(control_word));

    printf("Modes of Operation - Expected:      0x2f60600001000000\n");
    MotionControlMode_en motion_control_mode = MC_PROFILE_POSITION_MODE;
    CANOPEN.sdo_write(OBJ_Modes_of_Operation, 0, &motion_control_mode, sizeof(motion_control_mode));

    printf("Set Position Origin - Expected:     0x2302220000000000\n");
    int32_t position_origin = 0;
    CANOPEN.sdo_write(OBJ_Set_Position_Origin, 0, &position_origin, sizeof(position_origin));

    printf("Set Profile Vel - Expected:         0x2381600000000000\n");
    uint32_t profile_velocity = calculate_velocity(prof_vel);
    CANOPEN.sdo_write(OBJ_Profile_Velocity_in_PP_Mode, 0, &profile_velocity, sizeof(profile_velocity));

    printf("Set Profile Acc - Expected:         0x2383600000000000\n");
    uint32_t profile_accel = calculate_acceleration(prof_accel);
    CANOPEN.sdo_write(OBJ_Profile_Acceleration, 0, &profile_accel, sizeof(profile_accel));

    printf("Set Profile Dec - Expected:         0x2384600000000000\n");
    uint32_t profile_decel = calculate_acceleration(prof_decel);
    CANOPEN.sdo_write(OBJ_Profile_Deceleration, 0, &profile_decel, sizeof(profile_decel));

    printf("Set -ve Pos Lim - Expected:         0x2305220000000000\n");
    int32_t negative_pos_limit = -(int32_t)radians_to_encoder_count(20.0 * 3.1416 / 180.0);
    CANOPEN.sdo_write(OBJ_Negative_Software_Position_Limit, 0, &negative_pos_limit, sizeof(negative_pos_limit));

    printf("Set +ve Pos Lim - Expected:         0x2306220000000000\n");
    int32_t positive_pos_limit = radians_to_encoder_count(20.0 * 3.1416 / 180.0);
    CANOPEN.sdo_write(OBJ_Positive_Software_Position_Limit, 0, &positive_pos_limit, sizeof(positive_pos_limit));

    printf("Enable Pos Lims - Expected:         0x2b05230001000000\n");
    uint16_t software_limit_enable = 0b1U;
    CANOPEN.sdo_write(OBJ_Motor_Control, 0, &software_limit_enable, sizeof(software_limit_enable));
}

void send_motion_request(const double radians)
{
    uint16_t control_word  = 0U;
    uint8_t bytes_received = 0U;
    int32_t encoder_count  = radians_to_encoder_count(radians);
    encoder_count          = radians < 0 ? -encoder_count : encoder_count;
    CANOPEN.sdo_write(OBJ_Target_Position, 0, &encoder_count, sizeof(encoder_count));
    CANOPEN.sdo_read(OBJ_Control_Word, 0, nullptr, 0, &control_word, &bytes_received);
    SET_BIT_PATTERN(control_word, CW_NEW_SETPOINT);
    CANOPEN.sdo_write(OBJ_Control_Word, 0, &encoder_count, sizeof(encoder_count));
    CLEAR_BIT_PATTERN(control_word, CW_NEW_SETPOINT);
    CANOPEN.sdo_write(OBJ_Control_Word, 0, &encoder_count, sizeof(encoder_count));
}

void setup();

void loop();

int main(int argc, char **argv)
{
    setup();
    while (true)
    {
        loop();
    }

    return 0;
}

void setup()
{
    initMotor(1, 1, 1);
    usleep(1000U * 10U);
    drive_state_controller.set_target_state(DriveStateMachine::DS_SWITCH_ON_READY);
    usleep(1000U * 10U);
    drive_state_controller.set_target_state(DriveStateMachine::DS_SWITCHED_ON);
    usleep(1000U * 10U);
    drive_state_controller.set_target_state(DriveStateMachine::DS_OPERATION_ENABLED);
}

void loop()
{
    for (int i = -20; i <= 20; i++)
    {
        drive_state_controller.update();
        DriveStateMachine::DriveState_en current_state = drive_state_controller.get_current_state();
        if (current_state == DriveStateMachine::DS_OPERATION_ENABLED) { send_motion_request(i); }
        usleep(1000U * 10U);
    }
}

#endif
