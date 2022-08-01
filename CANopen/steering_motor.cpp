#include "CANopen.hpp"
#include "endian.hpp"
#include <cstddef>
#include <cstdlib>
#include <cstring>

constexpr uint8_t SM_NODE_ID = 0x4D;

void initMotor(uint32_t prof_vel, uint32_t prof_accel, uint32_t prof_decel)
{
    uint16_t cob_id = CANOPEN_FN_SDO_RX + SM_NODE_ID;

    can_raw_frame_t raw_frame;
    raw_frame.id = cob_id;

    /*Clear the faults in the Control Word (6040h)*/
    SDO_t clear_fault_sdo;
    clear_fault_sdo.ccs   = CCS_INITIATE_DOWNLOAD;
    clear_fault_sdo.e     = SDO_EXPIDITED_ENABLE;
    clear_fault_sdo.s     = SDO_SIZE_ENABLE;
    clear_fault_sdo.index = Control_Word;
    // ...
}

uint8_t rx_fn(struct can_frame * /*frame*/) { return 0; }

uint8_t tx_fn(const struct can_frame * /*frame*/) { return 0; }

const CANopen CANOPEN(rx_fn, tx_fn);

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

void setup() { initMotor(10, 11, 11); }

void loop() {}
