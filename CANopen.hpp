#pragma once

#include "CANopen_types.hpp"
#include "DriveStateMachine.hpp"
#include "autowp-mcp2515/can.h"
#include <stdint.h>

class CANopen
{
private:
    const uint8_t m_node_id;
    const can_rx_fn m_rx_function; // For passing in the MCP2515::sendMessage and readMessage
    const can_tx_fn m_tx_function;

public:
    // Rule of 5
    CANopen()                = delete;
    ~CANopen()               = default;
    CANopen(const CANopen &) = default;
    CANopen &operator=(const CANopen &) = delete;
    CANopen(CANopen &&)                 = default;
    CANopen &operator=(CANopen &&) = delete;

    CANopen(const uint8_t node_id, const can_rx_fn &rx_function, const can_tx_fn &tx_function)
        : m_node_id(node_id), m_rx_function(rx_function), m_tx_function(tx_function){};

    SDO_t sdo_read(CANopenObject_en object, uint8_t subindex, void *req_data, uint8_t req_data_length, void *rx_data,
                   uint8_t *rx_data_length) const;
    SDO_t sdo_write(CANopenObject_en object, uint8_t subindex, void *data, uint8_t data_length) const;
};
