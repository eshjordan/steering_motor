#pragma once

#include "CANopen_types.hpp"
#include "autowp-mcp2515/can.h"
#include <cstdint>

class CANopen
{
private:
    const can_rx_fn m_rx_function;
    const can_tx_fn m_tx_function;

public:
    CANopen();
    CANopen(can_rx_fn rx_fn, can_tx_fn tx_fn) : m_rx_function(rx_fn), m_tx_function(tx_fn) {}

    SDO_t sdo_read(SDO_t request);
    SDO_t sdo_write(SDO_t value);
};
