// typedef enum : uint16_t {

// } DriveState_en;

#pragma once

#include "CANopen_types.hpp"

class DriveStateMachine
{
public:
    enum DriveState_en : uint16_t {
        DS_NOT_READY             = SW_READY_TO_SWITCH_ON,
        DS_SWITCH_ON_DISABLED    = SW_SWITCH_ON_DISABLED,
        DS_SWITCH_ON_READY       = SW_QUICK_STOP | SW_READY_TO_SWITCH_ON,
        DS_SWITCHED_ON           = SW_QUICK_STOP | SW_SWITCHED_ON | SW_READY_TO_SWITCH_ON,
        DS_OPERATION_ENABLED     = SW_QUICK_STOP | SW_OPERATION_ENABLED | SW_SWITCHED_ON | SW_READY_TO_SWITCH_ON,
        DS_QUICK_STOP_ACTIVE     = SW_OPERATION_ENABLED | SW_SWITCHED_ON | SW_READY_TO_SWITCH_ON,
        DS_FAULT                 = SW_FAULT,
        DS_FAULT_REACTION_ACTIVE = SW_FAULT | SW_OPERATION_ENABLED | SW_SWITCHED_ON | SW_READY_TO_SWITCH_ON,
        DS_MAX                   = 0xFFFFU
    };

    bool set_state(const DriveState_en &state);

    inline DriveState_en get_state() { return m_current_state; }

    bool valid_transition(const DriveState_en &state, uint16_t *control_word);

    static DriveState_en calculate_state(uint16_t status_word);

private:
    DriveState_en m_current_state = DriveState_en::DS_NOT_READY;
};
