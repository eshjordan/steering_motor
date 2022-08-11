cmake_minimum_required(VERSION 3.10.2)

project(CANopen LANGUAGES C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_BUILD_TYPE "Debug")
add_compile_definitions(ENABLE_CPP_BUILD)

set(CXX_SRC
    "./CANopen.cpp"
    "./DriveStateController.cpp"
    "./DriveStateMachine.cpp"
    "./unit_conversions.cpp"
)

add_executable(CANopen
    "./steering_motor.cpp"
    ${CXX_SRC}
)

enable_testing()

add_executable(CANopen_test
    "./TestDriveStateMachine.cpp"
    ${CXX_SRC}
)

target_link_libraries(CANopen_test
    gtest
    gtest_main
    pthread
)

add_test(NAME CANopen_test COMMAND CANopen_test)
