#include "DriveStateMachine.hpp"

bool DriveStateMachine::valid_transition(const DriveState_en &state, uint16_t *control_word)
{
    bool ok_transition = false;
    switch (m_current_state)
    {
    case DriveState_en::DS_NOT_READY:
        switch (state)
        {
        // Valid transitions
        case DriveState_en::DS_SWITCH_ON_DISABLED:
        case DriveState_en::DS_FAULT_REACTION_ACTIVE: {
            ok_transition = true;
            *control_word = ControlWord_en::CW_MAX;
            break;
        }

        // Invalid transitions
        case DriveState_en::DS_NOT_READY:
        case DriveState_en::DS_SWITCH_ON_READY:
        case DriveState_en::DS_SWITCHED_ON:
        case DriveState_en::DS_OPERATION_ENABLED:
        case DriveState_en::DS_QUICK_STOP_ACTIVE:
        case DriveState_en::DS_FAULT:
        default: {
            ok_transition = false;
            *control_word = ControlWord_en::CW_MAX;
            break;
        }
        }
        break;
    case DriveState_en::DS_SWITCH_ON_DISABLED:
        switch (state)
        {
        // Valid transitions
        case DriveState_en::DS_SWITCH_ON_READY: {
            ok_transition = true;
            *control_word |= (uint16_t)(ControlWord_en::CW_QUICK_STOP | ControlWord_en::CW_ENABLE_VOLTAGE);
            break;
        }
        case DriveState_en::DS_FAULT_REACTION_ACTIVE: {
            ok_transition = true;
            *control_word = ControlWord_en::CW_MAX;
            break;
        }

        // Invalid transitions
        case DriveState_en::DS_SWITCH_ON_DISABLED:
        case DriveState_en::DS_NOT_READY:
        case DriveState_en::DS_SWITCHED_ON:
        case DriveState_en::DS_OPERATION_ENABLED:
        case DriveState_en::DS_QUICK_STOP_ACTIVE:
        case DriveState_en::DS_FAULT:
        default: {
            ok_transition = false;
            *control_word = ControlWord_en::CW_MAX;
            break;
        }
        }
        break;
    case DriveState_en::DS_SWITCH_ON_READY:
        switch (state)
        {
        // Valid transitions
        case DriveState_en::DS_SWITCHED_ON: {
            ok_transition = true;
            *control_word |= ControlWord_en::CW_SWITCH_ON;
            break;
        }
        case DriveState_en::DS_OPERATION_ENABLED: {
            ok_transition = true;
            *control_word |= (uint16_t)(ControlWord_en::CW_ENABLE_OPERATION | ControlWord_en::CW_SWITCH_ON);
            break;
        }
        case DriveState_en::DS_SWITCH_ON_DISABLED: {
            ok_transition = true;
            *control_word ^= (uint16_t)(ControlWord_en::CW_QUICK_STOP | ControlWord_en::CW_ENABLE_VOLTAGE);
            break;
        }
        case DriveState_en::DS_FAULT_REACTION_ACTIVE: {
            ok_transition = true;
            *control_word = ControlWord_en::CW_MAX;
            break;
        }

        // Invalid transitions
        case DriveState_en::DS_SWITCH_ON_READY:
        case DriveState_en::DS_NOT_READY:
        case DriveState_en::DS_QUICK_STOP_ACTIVE:
        case DriveState_en::DS_FAULT:
        default: {
            ok_transition = false;
            *control_word = ControlWord_en::CW_MAX;
            break;
        }
        }
        break;
    case DriveState_en::DS_SWITCHED_ON:
        switch (state)
        {
        // Valid transitions
        case DriveState_en::DS_SWITCH_ON_DISABLED: {
            ok_transition = true;
            *control_word ^= (uint16_t)(ControlWord_en::CW_QUICK_STOP | ControlWord_en::CW_ENABLE_VOLTAGE);
            break;
        }
        case DriveState_en::DS_SWITCH_ON_READY: {
            ok_transition = true;
            *control_word ^= ControlWord_en::CW_SWITCH_ON;
            break;
        }
        case DriveState_en::DS_OPERATION_ENABLED: {
            ok_transition = true;
            *control_word = ControlWord_en::CW_MAX;
            break;
        }
        case DriveState_en::DS_FAULT_REACTION_ACTIVE: {
            ok_transition = true;
            *control_word = ControlWord_en::CW_MAX;
            break;
        }

        // Invalid transitions
        case DriveState_en::DS_SWITCHED_ON:
        case DriveState_en::DS_NOT_READY:
        case DriveState_en::DS_QUICK_STOP_ACTIVE:
        case DriveState_en::DS_FAULT:
        default: {
            ok_transition = false;
            *control_word = ControlWord_en::CW_MAX;
            break;
        }
        }
        break;
    case DriveState_en::DS_OPERATION_ENABLED:
        switch (state)
        {
        // Valid transitions
        case DriveState_en::DS_SWITCH_ON_DISABLED: ok_transition = true; break;
        case DriveState_en::DS_SWITCH_ON_READY: ok_transition = true; break;
        case DriveState_en::DS_SWITCHED_ON: ok_transition = true; break;
        case DriveState_en::DS_QUICK_STOP_ACTIVE: ok_transition = true; break;
        case DriveState_en::DS_FAULT_REACTION_ACTIVE: {
            ok_transition = true;
            *control_word = ControlWord_en::CW_FAULT_REACTION_ACTIVE;
            break;
        }

        // Invalid transitions
        case DriveState_en::DS_OPERATION_ENABLED:
        case DriveState_en::DS_NOT_READY:
        case DriveState_en::DS_FAULT:
        default: {
            ok_transition = false;
            *control_word = ControlWord_en::CW_MAX;
            break;
        }
        }
        break;
    case DriveState_en::DS_QUICK_STOP_ACTIVE:
        switch (state)
        {
        // Valid transitions
        case DriveState_en::DS_SWITCH_ON_DISABLED: ok_transition = true; break;
        case DriveState_en::DS_FAULT_REACTION_ACTIVE: {
            ok_transition = true;
            *control_word = ControlWord_en::CW_FAULT_REACTION_ACTIVE;
            break;
        }

        // Invalid transitions
        case DriveState_en::DS_QUICK_STOP_ACTIVE:
        case DriveState_en::DS_NOT_READY:
        case DriveState_en::DS_SWITCH_ON_READY:
        case DriveState_en::DS_SWITCHED_ON:
        case DriveState_en::DS_OPERATION_ENABLED:
        case DriveState_en::DS_FAULT:
        default: {
            ok_transition = false;
            *control_word = ControlWord_en::CW_MAX;
            break;
        }
        }
        break;
    case DriveState_en::DS_FAULT:
        switch (state)
        {
        // Valid transitions
        case DriveState_en::DS_SWITCH_ON_DISABLED: ok_transition = true; break;

        // Invalid transitions
        case DriveState_en::DS_FAULT:
        case DriveState_en::DS_NOT_READY:
        case DriveState_en::DS_SWITCH_ON_READY:
        case DriveState_en::DS_SWITCHED_ON:
        case DriveState_en::DS_OPERATION_ENABLED:
        case DriveState_en::DS_QUICK_STOP_ACTIVE:
        case DriveState_en::DS_FAULT_REACTION_ACTIVE:
        default: {
            ok_transition = false;
            *control_word = ControlWord_en::CW_MAX;
            break;
        }
        }
        break;
    case DriveState_en::DS_FAULT_REACTION_ACTIVE:
        switch (state)
        {
        // Valid transitions
        case DriveState_en::DS_FAULT: ok_transition = true; break;

        // Invalid transitions
        case DriveState_en::DS_FAULT_REACTION_ACTIVE:
        case DriveState_en::DS_NOT_READY:
        case DriveState_en::DS_SWITCH_ON_DISABLED:
        case DriveState_en::DS_SWITCH_ON_READY:
        case DriveState_en::DS_SWITCHED_ON:
        case DriveState_en::DS_OPERATION_ENABLED:
        case DriveState_en::DS_QUICK_STOP_ACTIVE:
        default: {
            ok_transition = false;
            *control_word = ControlWord_en::CW_MAX;
            break;
        }
        }
        break;
    default: break;
    }

    return ok_transition;
}
