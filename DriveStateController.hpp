#pragma once

#include "CANopen.hpp"
#include "DriveStateMachine.hpp"

/**
 * @brief The DriveStateController class will automatically handle the state transitions required to enter a
 * particular target state.
 *
 */
class DriveStateController
{
private:
    using DriveState_en = DriveStateMachine::DriveState_en;

    /** @brief Communication object */
    const CANopen m_canopen;

    /** @brief Statemachine to validate transitions */
    DriveStateMachine m_drive_state_machine;

    /** @brief Target state of the drive */
    DriveState_en m_target_state{DriveStateMachine::DS_SWITCH_ON_READY};

protected:
    /**
     * @brief Safely request a state transition from one state to another.
     *
     * @param from The state to transition from.
     * @param to The state to transition to.
     * @return true if the state transition was successful.
     * @return false otherwise.
     */
    bool transition_state(const DriveState_en &from, const DriveState_en &to);

    /**
     * @brief Handle the state transition from DS_NOT_READY towards a target state.
     *
     * @return true if the state transition was successful.
     * @return false otherwise.
     */
    bool handle_not_ready();

    /**
     * @brief Handle the state transition from DS_SWITCH_ON_DISABLED towards a target state.
     *
     * @return true if the state transition was successful.
     * @return false otherwise.
     */
    bool handle_switch_on_disabled();

    /**
     * @brief Handle the state transition from DS_SWITCH_ON_READY towards a target state.
     *
     * @return true if the state transition was successful.
     * @return false otherwise.
     */
    bool handle_switch_on_ready();

    /**
     * @brief Handle the state transition from DS_SWITCHED_ON towards a target state.
     *
     * @return true if the state transition was successful.
     * @return false otherwise.
     */
    bool handle_switched_on();

    /**
     * @brief Handle the state transition from DS_OPERATION_ENABLED towards a target state.
     *
     * @return true if the state transition was successful.
     * @return false otherwise.
     */
    bool handle_operation_enabled();

    /**
     * @brief Handle the state transition from DS_FAULT towards a target state.
     *
     * @return true if the state transition was successful.
     * @return false otherwise.
     */
    bool handle_quick_stop_active();

    /**
     * @brief Handle the state transition from DS_FAULT towards a target state.
     *
     * @return true if the state transition was successful.
     * @return false otherwise.
     */
    bool handle_fault();

    /**
     * @brief Handle the state transition from DS_FAULT_REACTION_ACTIVE towards a target state.
     *
     * @return true if the state transition was successful.
     * @return false otherwise.
     */
    bool handle_fault_reaction_active();

    /**
     * @brief Read the Status Word from the motor and update the statemachine.
     *
     */
    void update_current_state()
    {
        uint8_t bytes_received = 0U;
        uint16_t status_word   = 0U;
        auto rx_sdo_1 = m_canopen.sdo_read(OBJ_Status_Word, 0, nullptr, 0, &status_word, &bytes_received);
        if (rx_sdo_1.reserved_0) {
            printf("Timeout waiting to read status word in update_current_state\n");
            return;
        }

        m_drive_state_machine.force_set_state(DriveStateMachine::calculate_state(status_word));
    }

public:
    /**
     * @brief Construct a new Drive State Controller object
     *
     * @param canopen The CANopen object to use for communication.
     * @param target_state The initial target state to transition towards.
     */
    explicit DriveStateController(const CANopen &canopen, const DriveState_en &target_state)
        : m_canopen(canopen), m_target_state(target_state){};

    /**
     * @brief Get the current target state.
     *
     * @return DriveState_en The current target state.
     */
    DriveState_en get_target_state() const { return m_target_state; }

    /**
     * @brief Set the target state.
     *
     * @param target_state The target state to transition towards. Must be one of DS_SWITCH_ON_READY, DS_SWITCHED_ON, or
     * DS_OPERATION_ENABLED.
     */
    bool set_target_state(const DriveState_en &target_state)
    {
        switch (target_state)
        {
        // Valid target states
        case DriveState_en::DS_SWITCH_ON_READY:
        case DriveState_en::DS_SWITCHED_ON:
        case DriveState_en::DS_OPERATION_ENABLED: {
            m_target_state = target_state;
            return true;
        }
        // Invalid target states
        case DriveState_en::DS_NOT_READY:
        case DriveState_en::DS_SWITCH_ON_DISABLED:
        case DriveState_en::DS_QUICK_STOP_ACTIVE:
        case DriveState_en::DS_FAULT:
        case DriveState_en::DS_FAULT_REACTION_ACTIVE:
        case DriveState_en::DS_MAX:
        default: {
            printf("Invalid target state\n");
            return false;
        }
        }
    }

    /**
     * @brief Get the current state from the motor.
     *
     * @return DriveState_en Motor state.
     */
    DriveState_en get_current_state()
    {
        update_current_state();
        return m_drive_state_machine.get_state();
    }

    /**
     * @brief Iteratively perform transitions towards the target state.
     *
     */
    void update();
};
