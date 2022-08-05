#pragma once

#include "CANopen_types.hpp"

class DriveStateMachine
{
public:
    /**
     * @brief Allowable states of the drive state machine.
     *
     */
    enum DriveState_en : uint16_t {
        DS_NOT_READY             = 0U,
        DS_SWITCH_ON_DISABLED    = SW_SWITCH_ON_DISABLED,
        DS_SWITCH_ON_READY       = SW_QUICK_STOP | SW_READY_TO_SWITCH_ON,
        DS_SWITCHED_ON           = SW_QUICK_STOP | SW_SWITCHED_ON | SW_READY_TO_SWITCH_ON,
        DS_OPERATION_ENABLED     = SW_QUICK_STOP | SW_OPERATION_ENABLED | SW_SWITCHED_ON | SW_READY_TO_SWITCH_ON,
        DS_QUICK_STOP_ACTIVE     = SW_OPERATION_ENABLED | SW_SWITCHED_ON | SW_READY_TO_SWITCH_ON,
        DS_FAULT                 = SW_FAULT,
        DS_FAULT_REACTION_ACTIVE = SW_FAULT | SW_OPERATION_ENABLED | SW_SWITCHED_ON | SW_READY_TO_SWITCH_ON,
        DS_MAX                   = 0xFFFFU
    };

    /**
     * @brief Status Word bitmasks for each Drive State
     *
     */
    enum DriveStateMask_en : uint16_t {
        DS_NOT_READY_MASK             = 0b0000000001001111U,
        DS_SWITCH_ON_DISABLED_MASK    = 0b0000000001001111U,
        DS_SWITCH_ON_READY_MASK       = 0b0000000001101111U,
        DS_SWITCHED_ON_MASK           = 0b0000000001101111U,
        DS_OPERATION_ENABLED_MASK     = 0b0000000001101111U,
        DS_QUICK_STOP_ACTIVE_MASK     = 0b0000000001101111U,
        DS_FAULT_MASK                 = 0b0000000001001111U,
        DS_FAULT_REACTION_ACTIVE_MASK = 0b0000000001001111U,
        DS_MAX_MASK                   = 0xFFFFU
    };

    static inline const char *drive_state_to_string(DriveState_en state)
    {
        switch (state)
        {
        case DS_NOT_READY: return "DS_NOT_READY";
        case DS_SWITCH_ON_DISABLED: return "DS_SWITCH_ON_DISABLED";
        case DS_SWITCH_ON_READY: return "DS_SWITCH_ON_READY";
        case DS_SWITCHED_ON: return "DS_SWITCHED_ON";
        case DS_OPERATION_ENABLED: return "DS_OPERATION_ENABLED";
        case DS_QUICK_STOP_ACTIVE: return "DS_QUICK_STOP_ACTIVE";
        case DS_FAULT: return "DS_FAULT";
        case DS_FAULT_REACTION_ACTIVE: return "DS_FAULT_REACTION_ACTIVE";
        case DS_MAX: return "DS_MAX";
        default: return "Unknown Drive State";
        }
    }

    /**
     * @brief Safely update the state of the drive, and set the Control Word used to perform the transition.
     * @see DriveStateMachine::valid_transition
     *
     * @param state New state to attempt to transition into
     * @param control_word Control word to be filled with the correct bits to perform the transition. If the transition
     * is invalid, or does not require sending a Control Word, this will be set to CW_MAX.
     * @return true if the transition is valid
     * @return false otherwise
     */
    bool set_state(const DriveState_en &state, uint16_t *control_word);

    /**
     * @brief Unsafely update the state of the drive, use when updating the state from a received Status Word.
     *
     * @param state New state to move to
     */
    inline void force_set_state(const DriveState_en &state) { m_current_state = state; };

    /**
     * @brief Get the current state of the drive.
     *
     * @return DriveState_en Current state of the drive.
     */
    inline DriveState_en get_state() { return m_current_state; }

    /**
     * @brief Given a new state, determine if a transition from the current state to the new one is valid, and if so,
     * set the Control Word used to perform the transition.
     *
     * @param initial_state State to transition from
     * @param final_state New state to attempt to transition into
     * @param control_word Control word to be filled with the correct bits to perform the transition. If the transition
     * is invalid, or does not require sending a Control Word, this will be set to CW_MAX.
     * @return true if the transition is valid
     * @return false otherwise
     */
    static bool valid_transition(const DriveState_en &initial_state, const DriveState_en &final_state,
                                 uint16_t *control_word);

    /**
     * @brief Given a two byte Status Word, determine the current state of the drive.
     *
     * @param status_word Status Word from the drive
     * @return DriveState_en State represented by the Status Word
     */
    static DriveState_en calculate_state(const uint16_t &status_word);

private:
    DriveState_en m_current_state = DriveState_en::DS_NOT_READY;
};
