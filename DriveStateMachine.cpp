#include "DriveStateMachine.hpp"

DriveStateMachine::DriveState_en DriveStateMachine::calculate_state(const uint16_t status_word)
{
    if ((status_word & SW_NOT_READY_MASK) == DriveStateMachine::DS_NOT_READY)
    {
        return DriveStateMachine::DS_NOT_READY;
    }
    if ((status_word & SW_SWITCH_ON_DISABLED_MASK) == DriveStateMachine::DS_SWITCH_ON_DISABLED)
    {
        return DriveStateMachine::DS_SWITCH_ON_DISABLED;
    }
    if ((status_word & SW_SWITCH_ON_READY_MASK) == DriveStateMachine::DS_SWITCH_ON_READY)
    {
        return DriveStateMachine::DS_SWITCH_ON_READY;
    }
    if ((status_word & SW_SWITCHED_ON_MASK) == DriveStateMachine::DS_SWITCHED_ON)
    {
        return DriveStateMachine::DS_SWITCHED_ON;
    }
    if ((status_word & SW_OPERATION_ENABLED_MASK) == DriveStateMachine::DS_OPERATION_ENABLED)
    {
        return DriveStateMachine::DS_OPERATION_ENABLED;
    }
    if ((status_word & SW_QUICK_STOP_ACTIVE_MASK) == DriveStateMachine::DS_QUICK_STOP_ACTIVE)
    {
        return DriveStateMachine::DS_QUICK_STOP_ACTIVE;
    }
    if ((status_word & SW_FAULT_MASK) == DriveStateMachine::DS_FAULT) { return DriveStateMachine::DS_FAULT; }
    if ((status_word & SW_FAULT_REACTION_ACTIVE_MASK) == DriveStateMachine::DS_FAULT_REACTION_ACTIVE)
    {
        return DriveStateMachine::DS_FAULT_REACTION_ACTIVE;
    }

    return DriveStateMachine::DS_MAX;
}

bool DriveStateMachine::valid_transition(const DriveState_en &initial_state, const DriveState_en &final_state,
                                         uint16_t *control_word)
{
    bool ok_transition = false;
    switch (initial_state)
    {
    case DriveState_en::DS_NOT_READY:
        switch (final_state)
        {
        // Valid transitions
        case DriveState_en::DS_SWITCH_ON_DISABLED:
        case DriveState_en::DS_FAULT_REACTION_ACTIVE: {
            ok_transition = true;
            SET_BIT_PATTERN(*control_word, ControlWord_en::CW_MAX);
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
            SET_BIT_PATTERN(*control_word, ControlWord_en::CW_MAX);
            break;
        }
        }
        break;
    case DriveState_en::DS_SWITCH_ON_DISABLED:
        switch (final_state)
        {
        // Valid transitions
        case DriveState_en::DS_SWITCH_ON_READY: {
            ok_transition = true;
            SET_BIT_PATTERN(*control_word, ControlWord_en::CW_QUICK_STOP | ControlWord_en::CW_ENABLE_VOLTAGE);
            break;
        }
        case DriveState_en::DS_FAULT_REACTION_ACTIVE: {
            ok_transition = true;
            SET_BIT_PATTERN(*control_word, ControlWord_en::CW_MAX);
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
            SET_BIT_PATTERN(*control_word, ControlWord_en::CW_MAX);
            break;
        }
        }
        break;
    case DriveState_en::DS_SWITCH_ON_READY:
        switch (final_state)
        {
        // Valid transitions
        case DriveState_en::DS_SWITCHED_ON: {
            ok_transition = true;
            SET_BIT_PATTERN(*control_word, ControlWord_en::CW_SWITCH_ON);
            break;
        }
        case DriveState_en::DS_OPERATION_ENABLED: {
            ok_transition = true;
            SET_BIT_PATTERN(*control_word, ControlWord_en::CW_ENABLE_OPERATION | ControlWord_en::CW_SWITCH_ON);
            break;
        }
        case DriveState_en::DS_SWITCH_ON_DISABLED: {
            ok_transition = true;
            CLEAR_BIT_PATTERN(*control_word, ControlWord_en::CW_QUICK_STOP | ControlWord_en::CW_ENABLE_VOLTAGE);
            break;
        }
        case DriveState_en::DS_FAULT_REACTION_ACTIVE: {
            ok_transition = true;
            SET_BIT_PATTERN(*control_word, ControlWord_en::CW_MAX);
            break;
        }

        // Invalid transitions
        case DriveState_en::DS_SWITCH_ON_READY:
        case DriveState_en::DS_NOT_READY:
        case DriveState_en::DS_QUICK_STOP_ACTIVE:
        case DriveState_en::DS_FAULT:
        default: {
            ok_transition = false;
            SET_BIT_PATTERN(*control_word, ControlWord_en::CW_MAX);
            break;
        }
        }
        break;
    case DriveState_en::DS_SWITCHED_ON:
        switch (final_state)
        {
        // Valid transitions
        case DriveState_en::DS_SWITCH_ON_DISABLED: {
            ok_transition = true;
            CLEAR_BIT_PATTERN(*control_word, ControlWord_en::CW_QUICK_STOP | ControlWord_en::CW_ENABLE_VOLTAGE);
            break;
        }
        case DriveState_en::DS_SWITCH_ON_READY: {
            ok_transition = true;
            CLEAR_BIT_PATTERN(*control_word, ControlWord_en::CW_SWITCH_ON);
            break;
        }
        case DriveState_en::DS_OPERATION_ENABLED: {
            ok_transition = true;
            SET_BIT_PATTERN(*control_word, ControlWord_en::CW_ENABLE_OPERATION);
            break;
        }
        case DriveState_en::DS_FAULT_REACTION_ACTIVE: {
            ok_transition = true;
            SET_BIT_PATTERN(*control_word, ControlWord_en::CW_MAX);
            break;
        }

        // Invalid transitions
        case DriveState_en::DS_SWITCHED_ON:
        case DriveState_en::DS_NOT_READY:
        case DriveState_en::DS_QUICK_STOP_ACTIVE:
        case DriveState_en::DS_FAULT:
        default: {
            ok_transition = false;
            SET_BIT_PATTERN(*control_word, ControlWord_en::CW_MAX);
            break;
        }
        }
        break;
    case DriveState_en::DS_OPERATION_ENABLED:
        switch (final_state)
        {
        // Valid transitions
        case DriveState_en::DS_SWITCH_ON_DISABLED: {
            ok_transition = true;
            CLEAR_BIT_PATTERN(*control_word, ControlWord_en::CW_ENABLE_VOLTAGE);
            break;
        }
        case DriveState_en::DS_SWITCH_ON_READY: {
            ok_transition = true;
            CLEAR_BIT_PATTERN(*control_word, ControlWord_en::CW_SWITCH_ON);
            break;
        }
        case DriveState_en::DS_SWITCHED_ON: {
            ok_transition = true;
            CLEAR_BIT_PATTERN(*control_word, ControlWord_en::CW_ENABLE_OPERATION);
            break;
        }
        case DriveState_en::DS_QUICK_STOP_ACTIVE: {
            ok_transition = true;
            CLEAR_BIT_PATTERN(*control_word, ControlWord_en::CW_QUICK_STOP);
            break;
        }
        case DriveState_en::DS_FAULT_REACTION_ACTIVE: {
            ok_transition = true;
            SET_BIT_PATTERN(*control_word, ControlWord_en::CW_MAX);
            break;
        }

        // Invalid transitions
        case DriveState_en::DS_OPERATION_ENABLED:
        case DriveState_en::DS_NOT_READY:
        case DriveState_en::DS_FAULT:
        default: {
            ok_transition = false;
            SET_BIT_PATTERN(*control_word, ControlWord_en::CW_MAX);
            break;
        }
        }
        break;
    case DriveState_en::DS_QUICK_STOP_ACTIVE:
        switch (final_state)
        {
        // Valid transitions
        case DriveState_en::DS_SWITCH_ON_DISABLED: ok_transition = true; break;
        case DriveState_en::DS_FAULT_REACTION_ACTIVE: {
            ok_transition = true;
            CLEAR_BIT_PATTERN(*control_word, ControlWord_en::CW_ENABLE_VOLTAGE);
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
            SET_BIT_PATTERN(*control_word, ControlWord_en::CW_MAX);
            break;
        }
        }
        break;
    case DriveState_en::DS_FAULT:
        switch (final_state)
        {
        // Valid transitions
        case DriveState_en::DS_SWITCH_ON_DISABLED: {
            ok_transition = true;
            SET_BIT_PATTERN(*control_word, ControlWord_en::CW_FAULT_RESET);
            break;
        }

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
            SET_BIT_PATTERN(*control_word, ControlWord_en::CW_MAX);
            break;
        }
        }
        break;
    case DriveState_en::DS_FAULT_REACTION_ACTIVE:
        switch (final_state)
        {
        // Valid transitions
        case DriveState_en::DS_FAULT: {
            ok_transition = true;
            SET_BIT_PATTERN(*control_word, ControlWord_en::CW_MAX);
            break;
        }

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
            SET_BIT_PATTERN(*control_word, ControlWord_en::CW_MAX);
            break;
        }
        }
        break;
    default: break;
    }

    return ok_transition;
}

bool DriveStateMachine::set_state(const DriveState_en &state, uint16_t *control_word)
{
    bool ok_transition = valid_transition(m_current_state, state, control_word);
    if (ok_transition) { m_current_state = state; }
    return ok_transition;
}
