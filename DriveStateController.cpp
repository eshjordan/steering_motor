#include "DriveStateController.hpp"
#include "CANopen.hpp"

bool DriveStateController::transition_state(const DriveState_en &from, const DriveState_en &to)
{
    uint8_t bytes_received = 0U;
    uint16_t control_word  = 0U;
    auto rx_sdo_1 = m_canopen.sdo_read(OBJ_Control_Word, 0, nullptr, 0, &control_word, &bytes_received);
    if (rx_sdo_1.reserved_0) {
        printf("Timeout waiting to read control word in transition_state\n");
        return false;
    }

    bool transition_ok = DriveStateMachine::valid_transition(from, to, &control_word);
    if (!transition_ok || control_word >= CW_MAX)
    {
        printf("Invalid transition from %s to %s\n", DriveStateMachine::drive_state_to_string(from),
               DriveStateMachine::drive_state_to_string(to));
        return false;
    }

    uint16_t status_word = 0U;
    auto rx_sdo_2 = m_canopen.sdo_write(OBJ_Control_Word, 0, &control_word, sizeof(control_word));
    if (rx_sdo_2.reserved_0) {
        printf("Timeout waiting to clear control word in transition_state\n");
        return false;
    }

    if (get_current_state() != DriveState_en::DS_SWITCHED_ON)
    {
        printf("Not transitioned from %s to %s\n", DriveStateMachine::drive_state_to_string(from),
               DriveStateMachine::drive_state_to_string(to));
        return false;
    }

    return true;
}

bool DriveStateController::handle_not_ready()
{
    printf("Drive State: NOT READY, still initialising\n");
    return false;
}

bool DriveStateController::handle_switch_on_disabled()
{
    switch (m_target_state)
    {
    case DriveState_en::DS_SWITCH_ON_READY:
    case DriveState_en::DS_SWITCHED_ON:
    case DriveState_en::DS_OPERATION_ENABLED: {
        printf("Drive State: SWITCH ON DISABLED, attempting to transition to SWITCH ON READY\n");
        return transition_state(DriveState_en::DS_SWITCH_ON_DISABLED, DriveState_en::DS_SWITCH_ON_READY);
    }
    default: {
        return false;
    }
    }
}

bool DriveStateController::handle_switch_on_ready()
{
    switch (m_target_state)
    {
    case DriveState_en::DS_SWITCH_ON_READY: {
        printf("Drive State: SWITCH ON READY, staying in SWITCH ON READY\n");
        return true;
    }
    case DriveState_en::DS_SWITCHED_ON:
    case DriveState_en::DS_OPERATION_ENABLED: {
        printf("Drive State: SWITCH ON READY, attempting to transition to SWITCHED ON\n");
        return transition_state(DriveState_en::DS_SWITCH_ON_READY, DriveState_en::DS_SWITCHED_ON);
    }
    default: {
        return false;
    }
    }
}

bool DriveStateController::handle_switched_on()
{
    switch (m_target_state)
    {
    case DriveState_en::DS_SWITCH_ON_READY:
    case DriveState_en::DS_SWITCHED_ON:
    case DriveState_en::DS_OPERATION_ENABLED: {
        printf("Drive State: SWITCHED ON, attempting to transition to %s\n",
               DriveStateMachine::drive_state_to_string(m_target_state));
        return transition_state(DriveState_en::DS_SWITCHED_ON, m_target_state);
    }
    default: {
        return false;
    }
    }
}

bool DriveStateController::handle_operation_enabled()
{
    switch (m_target_state)
    {
    case DriveState_en::DS_SWITCH_ON_READY:
    case DriveState_en::DS_SWITCHED_ON: {
        printf("Drive State: OPERATION ENABLED, attempting to transition to SWITCHED ON\n");
        return transition_state(DriveState_en::DS_OPERATION_ENABLED, DriveState_en::DS_SWITCHED_ON);
    }
    case DriveState_en::DS_OPERATION_ENABLED: {
        printf("Drive State: OPERATION ENABLED, staying in OPERATION ENABLED\n");
        return true;
    }
    default: {
        return false;
    }
    }
}

bool DriveStateController::handle_quick_stop_active()
{
    switch (m_target_state)
    {
    case DriveState_en::DS_SWITCH_ON_READY:
    case DriveState_en::DS_SWITCHED_ON:
    case DriveState_en::DS_OPERATION_ENABLED: {
        printf("Drive State: QUICK STOP ACTIVE, attempting to transition to SWITCH ON DISABLED\n");
        return transition_state(DriveState_en::DS_QUICK_STOP_ACTIVE, DriveState_en::DS_SWITCH_ON_DISABLED);
    }
    default: {
        return false;
    }
    }
}

bool DriveStateController::handle_fault()
{
    printf("Drive State: FAULT, attempting to transition to SWITCH ON DISABLED\n");
    return transition_state(DriveState_en::DS_FAULT, DriveState_en::DS_SWITCH_ON_DISABLED);
}

bool DriveStateController::handle_fault_reaction_active()
{
    printf("Drive State: FAULT REACTION ACTIVE, waiting for fault reaction to complete\n");
    return false;
}

void DriveStateController::update()
{
    using DriveState_en = DriveStateMachine::DriveState_en;

    switch (get_current_state())
    {
    case DriveState_en::DS_NOT_READY: {
        handle_not_ready();
        break;
    }
    case DriveState_en::DS_SWITCH_ON_DISABLED: {
        handle_switch_on_disabled();
        break;
    }
    case DriveState_en::DS_SWITCH_ON_READY: {
        handle_switch_on_ready();
        break;
    }
    case DriveState_en::DS_SWITCHED_ON: {
        handle_switched_on();
        break;
    }
    case DriveState_en::DS_OPERATION_ENABLED: {
        handle_operation_enabled();
        break;
    }
    case DriveState_en::DS_QUICK_STOP_ACTIVE: {
        handle_quick_stop_active();
        break;
    }
    case DriveState_en::DS_FAULT: {
        handle_fault();
        break;
    }
    case DriveState_en::DS_FAULT_REACTION_ACTIVE: {
        handle_fault_reaction_active();
        break;
    }
    default: break;
    }
}
