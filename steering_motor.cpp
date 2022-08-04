
#if (0)

#include "CANopen.hpp"
#include "endian.hpp"
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

constexpr uint8_t SM_NODE_ID = 0x4D;

uint8_t rx_fn(struct can_frame * /*frame*/, uint16_t) { return 0; }

uint8_t tx_fn(const struct can_frame *frame)
{
    printf("Sending CAN ID 0x%03lx, dlc: %hhu, data: ", frame->can_id, frame->can_dlc);
    print_can_frame_data(frame->data);
    return 0;
}

const CANopen CANOPEN(rx_fn, tx_fn);

void initMotor(uint32_t prof_vel, uint32_t prof_accel, uint32_t prof_decel)
{
    uint16_t cob_id = CANOPEN_FN_SDO_RX + SM_NODE_ID;

    /*Clear the faults in the Control Word (6040h)*/
    printf("Clear Faults - Expected:            0x2b40600000000000\n");
    ControlWord_en control_word = CW_FAULT_RESET;
    CANOPEN.sdo_write(SM_NODE_ID, OBJ_Control_Word, 0, &control_word, sizeof(control_word));

    printf("Modes of Operation - Expected:      0x2f60600001000000\n");
    MotionControlMode_en motion_control_mode = MC_PROFILE_POSITION_MODE;
    CANOPEN.sdo_write(SM_NODE_ID, OBJ_Modes_of_Operation, 0, &motion_control_mode, sizeof(motion_control_mode));

    printf("Set Position Origin - Expected:     0x2302220000000000\n");
    int32_t position_origin = 0;
    CANOPEN.sdo_write(SM_NODE_ID, OBJ_Set_Position_Origin, 0, &position_origin, sizeof(position_origin));

    printf("Set Profile Vel - Expected:         0x2381600000000000\n");
    uint32_t profile_velocity = calculate_velocity(prof_vel);
    CANOPEN.sdo_write(SM_NODE_ID, OBJ_Profile_Velocity_in_PP_Mode, 0, &profile_velocity, sizeof(profile_velocity));

    // //Set Profile Velocity in PP Mode object (6081h)
    // can_data = 0x2381600000000000 + integerToLittleEnd(velocityTarget);
    // sendMessageCAN(sdoReceiveCobId, 8, can_data);
    // delay(10);

    // //Set Profile Aceleration (6083h)
    // can_data = 0x2383600000000000 + integerToLittleEnd(accelTarget);
    // sendMessageCAN(sdoReceiveCobId, 8, can_data);
    // delay(10);

    // //Set Profile Deceleration (6083h)
    // can_data = 0x2384600000000000 + integerToLittleEnd(decelTarget);
    // sendMessageCAN(sdoReceiveCobId, 8, can_data);
    // delay(10);

    // //Set Negative software position limit (2205h)
    // can_data = 0x2305220000000000 + integerToLittleEnd(minEncoderIncrement);
    // sendMessageCAN(sdoReceiveCobId, 8, can_data);
    // delay(10);

    // //Set Positive software position limit (2206h)
    // can_data = 0x2306220000000000 + integerToLittleEnd(maxEncoderIncrement);
    // sendMessageCAN(sdoReceiveCobId, 8, can_data);
    // delay(10);

    // ...
}

void setup();

void loop();

int main(int argc, char **argv)
{
    setup();
    while (false)
    {
        loop();
    }

    return 0;
}

void setup() { initMotor(1, 1, 1); }

void loop() {}

#endif
