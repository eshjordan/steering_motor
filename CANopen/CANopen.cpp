
#include "CANopen.hpp"
#include "CANopen_types.hpp"
#include "autowp-mcp2515/can.h"
#include "endian.hpp"
#include <cstdint>
#include <cstdio>
#include <cstring>

SDO_t CANopen::sdo_read(uint16_t node_id, uint16_t object, uint8_t subindex, uint8_t *req_data, uint8_t req_data_length,
                        uint8_t *rx_data, uint8_t *rx_data_length) const
{
    //Determine COB ID for slave to transmit.
    uint16_t rd_cob_id = node_id + CANOPEN_FN_SDO_TX;
    //Determine COB ID for master receive from slave
    uint16_t wr_cob_id = node_id + CANOPEN_FN_SDO_RX;

    //Generate an SDO message for request.
    SDO_t sdo_msg{.data = {0}};
    sdo_msg.ccs      = CCS_INITIATE_UPLOAD;
    sdo_msg.n        = sizeof(SDO_t::data) - req_data_length;
    sdo_msg.e        = SDO_EXPEDITED_ENABLE;
    sdo_msg.s        = SDO_SIZE_ENABLE;
    sdo_msg.index    = object;
    sdo_msg.subindex = subindex;
    mempcpy(sdo_msg.data, req_data, req_data_length);

    //Generate a request CAN frame from SDO message.
    can_frame tx_frame = {0};
    tx_frame.can_id    = wr_cob_id;
    mempcpy(tx_frame.data, &sdo_msg, sizeof(SDO_t));
    tx_frame.can_dlc = sizeof(SDO_t);
    m_tx_function(&tx_frame); //Transmit request

    //Generate a CAN frame to be populated with received frame.
    can_frame rx_frame = {0};


    m_rx_function(&rx_frame, rd_cob_id);
    
    //Extract data from received frame to SDO message.
    SDO_t rx_sdo_msg{.data = {0}};
    mempcpy(&rx_sdo_msg, rx_frame.data, sizeof(SDO_t));
    *rx_data_length      = sizeof(SDO_t::data) - rx_sdo_msg.n;
    *(uint32_t *)rx_data = swap_uint32(*(uint32_t *)rx_sdo_msg.data);

    return rx_sdo_msg;
}

SDO_t CANopen::sdo_write(uint8_t node_id, uint16_t object, uint8_t subindex, uint8_t *data, uint8_t data_length) const
{
    //Determine COB ID for slave receive
    uint16_t wr_cob_id = node_id + CANOPEN_FN_SDO_RX;
    //Determine COB ID for slave transmit
    uint16_t rd_cob_id = node_id + CANOPEN_FN_SDO_TX;

    SDO_t sdo_msg{.data = {0}};
    sdo_msg.ccs   = CCS_INITIATE_DOWNLOAD;
    sdo_msg.n     = sizeof(SDO_t::data) - data_length;
    sdo_msg.e     = SDO_EXPEDITED_ENABLE;
    sdo_msg.s     = SDO_SIZE_ENABLE;
    sdo_msg.index = object;
    // sdo_msg.index    = swap_uint16(object);
    sdo_msg.subindex = subindex;
    mempcpy(sdo_msg.data, data, data_length);
    // *(uint32_t *)sdo_msg.data = swap_uint32(*(uint32_t *)sdo_msg.data);

    //Generate a can frame to populate and transmit.
    can_frame tx_frame = {0};
    tx_frame.can_id    = wr_cob_id;
    mempcpy(tx_frame.data, &sdo_msg, sizeof(SDO_t));
    tx_frame.can_dlc = sizeof(SDO_t);
    print_frame_data(tx_frame.data);
    m_tx_function(&tx_frame); //Transmit frame

    //Generate a CAN frame and listen for response
    can_frame rx_frame = {0};
    m_rx_function(&rx_frame, rd_cob_id);
    return {};

}

StatusWord_t CANopen::get_status_word(uint16_t value)
{
    if ((value & SW_NOT_READY_MASK) == SW_NOT_READY) { return SW_NOT_READY; }
    if ((value & SW_SWITCH_ON_DISABLED_MASK) == SW_SWITCH_ON_DISABLED) { return SW_SWITCH_ON_DISABLED; }
    if ((value & SW_SWITCH_ON_READY_MASK) == SW_SWITCH_ON_READY) { return SW_SWITCH_ON_READY; }
    if ((value & SW_SWITCHED_ON_MASK) == SW_SWITCHED_ON) { return SW_SWITCHED_ON; }
    if ((value & SW_OPERATION_ENABLED_MASK) == SW_OPERATION_ENABLED) { return SW_OPERATION_ENABLED; }
    if ((value & SW_QUICK_STOP_ACTIVE_MASK) == SW_QUICK_STOP_ACTIVE) { return SW_QUICK_STOP_ACTIVE; }
    if ((value & SW_FAULT_MASK) == SW_FAULT) { return SW_FAULT; }
    if ((value & SW_FAULT_REACTION_ACTIVE_MASK) == SW_FAULT_REACTION_ACTIVE) { return SW_FAULT_REACTION_ACTIVE; }

    return SW_ERROR;
}
