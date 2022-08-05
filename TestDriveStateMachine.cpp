
#if defined ENABLE_CPP_BUILD

#include "DriveStateMachine.hpp"
#include "gtest/gtest.h"

class TestDriveStateMachine : public ::testing::Test
{
protected:
    void SetUp() override {}

    void TearDown() override {}
};

/**
 * @brief Test that the DriveStateMachine::calculate_state function correctly determines which Drive State a status word
 * represents, ignoring unmasked bit values.
 *
 */
TEST_F(TestDriveStateMachine, testCalculateState)
{
    uint16_t status_word = 0b1111111110110000U;
    ASSERT_EQ(DriveStateMachine::DS_NOT_READY, DriveStateMachine::calculate_state(status_word));
    status_word = 0b1111111111110000U;
    ASSERT_EQ(DriveStateMachine::DS_SWITCH_ON_DISABLED, DriveStateMachine::calculate_state(status_word));
    status_word = 0b1111111110110001U;
    ASSERT_EQ(DriveStateMachine::DS_SWITCH_ON_READY, DriveStateMachine::calculate_state(status_word));
    status_word = 0b1111111110110011U;
    ASSERT_EQ(DriveStateMachine::DS_SWITCHED_ON, DriveStateMachine::calculate_state(status_word));
    status_word = 0b1111111110110111U;
    ASSERT_EQ(DriveStateMachine::DS_OPERATION_ENABLED, DriveStateMachine::calculate_state(status_word));
    status_word = 0b1111111110010111U;
    ASSERT_EQ(DriveStateMachine::DS_QUICK_STOP_ACTIVE, DriveStateMachine::calculate_state(status_word));
    status_word = 0b1111111110111111U;
    ASSERT_EQ(DriveStateMachine::DS_FAULT_REACTION_ACTIVE, DriveStateMachine::calculate_state(status_word));
    status_word = 0b1111111110111000U;
    ASSERT_EQ(DriveStateMachine::DS_FAULT, DriveStateMachine::calculate_state(status_word));
    status_word = 0b1110111111111111U;
    ASSERT_EQ(DriveStateMachine::DS_MAX, DriveStateMachine::calculate_state(status_word));
}

TEST_F(TestDriveStateMachine, testDriveStateToString)
{
    ASSERT_STREQ("DS_NOT_READY", DriveStateMachine::drive_state_to_string(DriveStateMachine::DS_NOT_READY));
    ASSERT_STREQ("DS_SWITCH_ON_DISABLED",
                 DriveStateMachine::drive_state_to_string(DriveStateMachine::DS_SWITCH_ON_DISABLED));
    ASSERT_STREQ("DS_SWITCH_ON_READY", DriveStateMachine::drive_state_to_string(DriveStateMachine::DS_SWITCH_ON_READY));
    ASSERT_STREQ("DS_SWITCHED_ON", DriveStateMachine::drive_state_to_string(DriveStateMachine::DS_SWITCHED_ON));
    ASSERT_STREQ("DS_OPERATION_ENABLED",
                 DriveStateMachine::drive_state_to_string(DriveStateMachine::DS_OPERATION_ENABLED));
    ASSERT_STREQ("DS_QUICK_STOP_ACTIVE",
                 DriveStateMachine::drive_state_to_string(DriveStateMachine::DS_QUICK_STOP_ACTIVE));
    ASSERT_STREQ("DS_FAULT_REACTION_ACTIVE",
                 DriveStateMachine::drive_state_to_string(DriveStateMachine::DS_FAULT_REACTION_ACTIVE));
    ASSERT_STREQ("DS_FAULT", DriveStateMachine::drive_state_to_string(DriveStateMachine::DS_FAULT));
    ASSERT_STREQ("DS_MAX", DriveStateMachine::drive_state_to_string(DriveStateMachine::DS_MAX));
}

/**
 * @brief Test state transitions and control word values produced.
 * Not implemented yet.
 *
 */
TEST_F(TestDriveStateMachine, testValidTransitions)
{
    using DSM_en = DriveStateMachine::DriveState_en;

    ControlWord_en control_word = (ControlWord_en)0;
    bool transition_ok = DriveStateMachine::valid_transition(DSM_en::DS_NOT_READY, DSM_en::DS_SWITCH_ON_DISABLED,
                                                             (uint16_t *)&control_word);
    printf(transition_ok ? "DS_NOT_READY to DS_SWITCH_ON_DISABLED OK\n"
                         : "DS_NOT_READY to DS_SWITCH_ON_DISABLED NOT OK\n");
    printf("Control Word: 0x%04x\n", control_word);

    control_word  = (ControlWord_en)0;
    transition_ok = DriveStateMachine::valid_transition(DSM_en::DS_SWITCH_ON_DISABLED, DSM_en::DS_SWITCH_ON_READY,
                                                        (uint16_t *)&control_word);
    printf(transition_ok ? "DS_SWITCH_ON_DISABLED to DS_SWITCH_ON_READY OK\n"
                         : "DS_SWITCH_ON_DISABLED to DS_SWITCH_ON_READY NOT OK\n");
    printf("Control Word: 0x%04x\n", control_word);

    control_word  = (ControlWord_en)0;
    transition_ok = DriveStateMachine::valid_transition(DSM_en::DS_SWITCH_ON_READY, DSM_en::DS_SWITCHED_ON,
                                                        (uint16_t *)&control_word);
    printf(transition_ok ? "DS_SWITCH_ON_READY to DS_SWITCHED_ON OK\n"
                         : "DS_SWITCH_ON_READY to DS_SWITCHED_ON NOT OK\n");
    printf("Control Word: 0x%04x\n", control_word);

    control_word  = (ControlWord_en)0;
    transition_ok = DriveStateMachine::valid_transition(DSM_en::DS_SWITCHED_ON, DSM_en::DS_OPERATION_ENABLED,
                                                        (uint16_t *)&control_word);
    printf(transition_ok ? "DS_SWITCHED_ON to DS_OPERATION_ENABLED OK\n"
                         : "DS_SWITCHED_ON to DS_OPERATION_ENABLED NOT OK\n");
    printf("Control Word: 0x%04x\n", control_word);

    control_word  = (ControlWord_en)0xFFFFU;
    transition_ok = DriveStateMachine::valid_transition(DSM_en::DS_OPERATION_ENABLED, DSM_en::DS_QUICK_STOP_ACTIVE,
                                                        (uint16_t *)&control_word);
    printf(transition_ok ? "DS_OPERATION_ENABLED to DS_QUICK_STOP_ACTIVE OK\n"
                         : "DS_OPERATION_ENABLED to DS_QUICK_STOP_ACTIVE NOT OK\n");
    printf("Control Word: 0x%04x\n", control_word);

    control_word  = (ControlWord_en)0xFFFFU;
    transition_ok = DriveStateMachine::valid_transition(DSM_en::DS_QUICK_STOP_ACTIVE, DSM_en::DS_SWITCH_ON_DISABLED,
                                                        (uint16_t *)&control_word);
    printf(transition_ok ? "DS_QUICK_STOP_ACTIVE to DS_SWITCH_ON_DISABLED OK\n"
                         : "DS_QUICK_STOP_ACTIVE to DS_SWITCH_ON_DISABLED NOT OK\n");
    printf("Control Word: 0x%04x\n", control_word);
}

#endif
