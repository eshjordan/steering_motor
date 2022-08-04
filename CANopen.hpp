#pragma once

#include "CANopen_types.hpp"
#include "DriveStateMachine.hpp"
#include "autowp-mcp2515/can.h"
#include <stdint.h>

class CANopen
{
private:
    const can_rx_fn m_rx_function; // For passing in the MCP2515::sendMessage and readMessage
    const can_tx_fn m_tx_function;

public:
    CANopen();
    CANopen(can_rx_fn rx_fn, can_tx_fn tx_fn) : m_rx_function(rx_fn), m_tx_function(tx_fn) {}

    SDO_t sdo_read(uint8_t node_id, CANopenObject_en object, uint8_t subindex, void *req_data, uint8_t req_data_length,
                   void *rx_data, uint8_t *rx_data_length) const;
    SDO_t sdo_write(uint8_t node_id, CANopenObject_en object, uint8_t subindex, void *data, uint8_t data_length) const;
};
