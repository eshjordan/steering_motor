
#include "CANopen.hpp"
#include "CANopen_types.hpp"
#include "autowp-mcp2515/can.h"
#include "endian.hpp"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

SDO_t CANopen::sdo_read(uint8_t node_id, CANopenObject_en object, uint8_t subindex, void *req_data,
                        uint8_t req_data_length, void *rx_data, uint8_t *rx_data_length) const
{
    // Determine COB ID for slave to transmit.
    const uint16_t request_cob_id = node_id + CANOPEN_FN_SDO_TX;

    // Determine COB ID for master receive from slave
    const uint16_t result_cob_id = node_id + CANOPEN_FN_SDO_RX;

    // Generate an SDO message for request.
    SDO_t sdo_msg    = {0};
    sdo_msg.ccs      = CCS_INITIATE_UPLOAD;
    sdo_msg.n        = sizeof(SDO_t::data) - req_data_length;
    sdo_msg.e        = SDO_EXPEDITED_ENABLE;
    sdo_msg.s        = SDO_SIZE_ENABLE;
    sdo_msg.index    = object;
    sdo_msg.subindex = subindex;
    memcpy(sdo_msg.data, req_data, req_data_length);

    // Generate a request CAN frame from SDO message.
    can_frame tx_frame = {0};
    tx_frame.can_id    = request_cob_id;
    memcpy(tx_frame.data, &sdo_msg, sizeof(SDO_t));
    tx_frame.can_dlc = sizeof(SDO_t);
    m_tx_function(&tx_frame); // Transmit request

    // Generate a CAN frame to be populated with received frame.
    can_frame rx_frame = {0};

    m_rx_function(&rx_frame, result_cob_id);

    // Extract data from received frame to SDO message.
    SDO_t rx_sdo_msg = {0};
    memcpy(&rx_sdo_msg, rx_frame.data, sizeof(SDO_t));
    *rx_data_length = sizeof(SDO_t::data) - rx_sdo_msg.n;

    return rx_sdo_msg;
}

SDO_t CANopen::sdo_write(uint8_t node_id, CANopenObject_en object, uint8_t subindex, void *data,
                         uint8_t data_length) const
{
    // Determine COB ID for slave receive
    const uint16_t request_cob_id = node_id + CANOPEN_FN_SDO_RX;

    // Determine COB ID for slave transmit
    const uint16_t result_cob_id = node_id + CANOPEN_FN_SDO_TX;

    SDO_t sdo_msg    = {0};
    sdo_msg.ccs      = CCS_INITIATE_DOWNLOAD;
    sdo_msg.n        = sizeof(SDO_t::data) - data_length;
    sdo_msg.e        = SDO_EXPEDITED_ENABLE;
    sdo_msg.s        = SDO_SIZE_ENABLE;
    sdo_msg.index    = object;
    sdo_msg.subindex = subindex;
    memcpy(sdo_msg.data, data, data_length);

    // Generate a can frame to populate and transmit.
    can_frame tx_frame = {0};
    tx_frame.can_id    = request_cob_id;
    memcpy(tx_frame.data, &sdo_msg, sizeof(SDO_t));
    tx_frame.can_dlc = sizeof(SDO_t);
    m_tx_function(&tx_frame); // Transmit frame

    // Generate a CAN frame and listen for response
    can_frame rx_frame = {0};
    m_rx_function(&rx_frame, result_cob_id);
    return {};
}
