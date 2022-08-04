#include "DriveStateMachine.hpp"

DriveStateMachine::DriveState_en DriveStateMachine::calculate_state(const uint16_t status_word)
{
    if ((status_word & DS_NOT_READY_MASK) == DS_NOT_READY) { return DS_NOT_READY; }
    if ((status_word & DS_SWITCH_ON_DISABLED_MASK) == DS_SWITCH_ON_DISABLED) { return DS_SWITCH_ON_DISABLED; }
    if ((status_word & DS_SWITCH_ON_READY_MASK) == DS_SWITCH_ON_READY) { return DS_SWITCH_ON_READY; }
    if ((status_word & DS_SWITCHED_ON_MASK) == DS_SWITCHED_ON) { return DS_SWITCHED_ON; }
    if ((status_word & DS_OPERATION_ENABLED_MASK) == DS_OPERATION_ENABLED) { return DS_OPERATION_ENABLED; }
    if ((status_word & DS_QUICK_STOP_ACTIVE_MASK) == DS_QUICK_STOP_ACTIVE) { return DS_QUICK_STOP_ACTIVE; }
    if ((status_word & DS_FAULT_MASK) == DS_FAULT) { return DS_FAULT; }
    if ((status_word & DS_FAULT_REACTION_ACTIVE_MASK) == DS_FAULT_REACTION_ACTIVE) { return DS_FAULT_REACTION_ACTIVE; }

    return DS_MAX;
}

bool DriveStateMachine::valid_transition(const DriveState_en &initial_state, const DriveState_en &final_state,
                                         uint16_t *control_word)
{
    bool ok_transition = false;
    switch (initial_state)
    {
    case DS_NOT_READY:
        switch (final_state)
        {
        // Valid transitions
        case DS_SWITCH_ON_DISABLED:
        case DS_FAULT_REACTION_ACTIVE: {
            ok_transition = true;
            SET_BIT_PATTERN(*control_word, CW_MAX);
            break;
        }

        // Invalid transitions
        case DS_NOT_READY:
        case DS_SWITCH_ON_READY:
        case DS_SWITCHED_ON:
        case DS_OPERATION_ENABLED:
        case DS_QUICK_STOP_ACTIVE:
        case DS_FAULT:
        default: {
            ok_transition = false;
            SET_BIT_PATTERN(*control_word, CW_MAX);
            break;
        }
        }
        break;
    case DS_SWITCH_ON_DISABLED:
        switch (final_state)
        {
        // Valid transitions
        case DS_SWITCH_ON_READY: {
            ok_transition = true;
            SET_BIT_PATTERN(*control_word, CW_QUICK_STOP | CW_ENABLE_VOLTAGE);
            break;
        }
        case DS_FAULT_REACTION_ACTIVE: {
            ok_transition = true;
            SET_BIT_PATTERN(*control_word, CW_MAX);
            break;
        }

        // Invalid transitions
        case DS_SWITCH_ON_DISABLED:
        case DS_NOT_READY:
        case DS_SWITCHED_ON:
        case DS_OPERATION_ENABLED:
        case DS_QUICK_STOP_ACTIVE:
        case DS_FAULT:
        default: {
            ok_transition = false;
            SET_BIT_PATTERN(*control_word, CW_MAX);
            break;
        }
        }
        break;
    case DS_SWITCH_ON_READY:
        switch (final_state)
        {
        // Valid transitions
        case DS_SWITCHED_ON: {
            ok_transition = true;
            SET_BIT_PATTERN(*control_word, CW_SWITCH_ON);
            break;
        }
        case DS_OPERATION_ENABLED: {
            ok_transition = true;
            SET_BIT_PATTERN(*control_word, CW_ENABLE_OPERATION | CW_SWITCH_ON);
            break;
        }
        case DS_SWITCH_ON_DISABLED: {
            ok_transition = true;
            CLEAR_BIT_PATTERN(*control_word, CW_QUICK_STOP | CW_ENABLE_VOLTAGE);
            break;
        }
        case DS_FAULT_REACTION_ACTIVE: {
            ok_transition = true;
            SET_BIT_PATTERN(*control_word, CW_MAX);
            break;
        }

        // Invalid transitions
        case DS_SWITCH_ON_READY:
        case DS_NOT_READY:
        case DS_QUICK_STOP_ACTIVE:
        case DS_FAULT:
        default: {
            ok_transition = false;
            SET_BIT_PATTERN(*control_word, CW_MAX);
            break;
        }
        }
        break;
    case DS_SWITCHED_ON:
        switch (final_state)
        {
        // Valid transitions
        case DS_SWITCH_ON_DISABLED: {
            ok_transition = true;
            CLEAR_BIT_PATTERN(*control_word, CW_QUICK_STOP | CW_ENABLE_VOLTAGE);
            break;
        }
        case DS_SWITCH_ON_READY: {
            ok_transition = true;
            CLEAR_BIT_PATTERN(*control_word, CW_SWITCH_ON);
            break;
        }
        case DS_OPERATION_ENABLED: {
            ok_transition = true;
            SET_BIT_PATTERN(*control_word, CW_ENABLE_OPERATION);
            break;
        }
        case DS_FAULT_REACTION_ACTIVE: {
            ok_transition = true;
            SET_BIT_PATTERN(*control_word, CW_MAX);
            break;
        }

        // Invalid transitions
        case DS_SWITCHED_ON:
        case DS_NOT_READY:
        case DS_QUICK_STOP_ACTIVE:
        case DS_FAULT:
        default: {
            ok_transition = false;
            SET_BIT_PATTERN(*control_word, CW_MAX);
            break;
        }
        }
        break;
    case DS_OPERATION_ENABLED:
        switch (final_state)
        {
        // Valid transitions
        case DS_SWITCH_ON_DISABLED: {
            ok_transition = true;
            CLEAR_BIT_PATTERN(*control_word, CW_ENABLE_VOLTAGE);
            break;
        }
        case DS_SWITCH_ON_READY: {
            ok_transition = true;
            CLEAR_BIT_PATTERN(*control_word, CW_SWITCH_ON);
            break;
        }
        case DS_SWITCHED_ON: {
            ok_transition = true;
            CLEAR_BIT_PATTERN(*control_word, CW_ENABLE_OPERATION);
            break;
        }
        case DS_QUICK_STOP_ACTIVE: {
            ok_transition = true;
            CLEAR_BIT_PATTERN(*control_word, CW_QUICK_STOP);
            break;
        }
        case DS_FAULT_REACTION_ACTIVE: {
            ok_transition = true;
            SET_BIT_PATTERN(*control_word, CW_MAX);
            break;
        }

        // Invalid transitions
        case DS_OPERATION_ENABLED:
        case DS_NOT_READY:
        case DS_FAULT:
        default: {
            ok_transition = false;
            SET_BIT_PATTERN(*control_word, CW_MAX);
            break;
        }
        }
        break;
    case DS_QUICK_STOP_ACTIVE:
        switch (final_state)
        {
        // Valid transitions
        case DS_SWITCH_ON_DISABLED: {
            ok_transition = true;
            CLEAR_BIT_PATTERN(*control_word, CW_ENABLE_VOLTAGE);
            break;
        }
        case DS_FAULT_REACTION_ACTIVE: {
            ok_transition = true;
            SET_BIT_PATTERN(*control_word, CW_MAX);
            break;
        }

        // Invalid transitions
        case DS_QUICK_STOP_ACTIVE:
        case DS_NOT_READY:
        case DS_SWITCH_ON_READY:
        case DS_SWITCHED_ON:
        case DS_OPERATION_ENABLED:
        case DS_FAULT:
        default: {
            ok_transition = false;
            SET_BIT_PATTERN(*control_word, CW_MAX);
            break;
        }
        }
        break;
    case DS_FAULT:
        switch (final_state)
        {
        // Valid transitions
        case DS_SWITCH_ON_DISABLED: {
            ok_transition = true;
            SET_BIT_PATTERN(*control_word, CW_FAULT_RESET);
            break;
        }

        // Invalid transitions
        case DS_FAULT:
        case DS_NOT_READY:
        case DS_SWITCH_ON_READY:
        case DS_SWITCHED_ON:
        case DS_OPERATION_ENABLED:
        case DS_QUICK_STOP_ACTIVE:
        case DS_FAULT_REACTION_ACTIVE:
        default: {
            ok_transition = false;
            SET_BIT_PATTERN(*control_word, CW_MAX);
            break;
        }
        }
        break;
    case DS_FAULT_REACTION_ACTIVE:
        switch (final_state)
        {
        // Valid transitions
        case DS_FAULT: {
            ok_transition = true;
            SET_BIT_PATTERN(*control_word, CW_MAX);
            break;
        }

        // Invalid transitions
        case DS_FAULT_REACTION_ACTIVE:
        case DS_NOT_READY:
        case DS_SWITCH_ON_DISABLED:
        case DS_SWITCH_ON_READY:
        case DS_SWITCHED_ON:
        case DS_OPERATION_ENABLED:
        case DS_QUICK_STOP_ACTIVE:
        default: {
            ok_transition = false;
            SET_BIT_PATTERN(*control_word, CW_MAX);
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
