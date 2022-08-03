#pragma once

#include "CANopen_types.hpp"
#include "autowp-mcp2515/can.h"
#include <stdint.h>

class CANopen
{
private:
    const can_rx_fn m_rx_function; //For passing in the MCP2515::sendMessage and readMessage
    const can_tx_fn m_tx_function;


public:
    CANopen();
    CANopen(can_rx_fn rx_fn, can_tx_fn tx_fn) : m_rx_function(rx_fn), m_tx_function(tx_fn) {}

    SDO_t sdo_read(uint16_t node_id, uint16_t object, uint8_t subindex, uint8_t *req_data, uint8_t req_data_length, uint8_t *rx_data, uint8_t *rx_data_length) const;
    SDO_t sdo_write(uint8_t node_id, uint16_t object, uint8_t subindex, uint8_t *data, uint8_t data_length) const;

    StatusWord_t get_status_word(uint16_t value);
};
