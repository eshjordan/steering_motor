cmake_minimum_required(VERSION 3.10.2)

project(CANopen LANGUAGES C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-std=c++2a -g)
set(CMAKE_BUILD_TYPE "Debug")

add_executable(CANopen
    "./steering_motor.cpp"
    "./CANopen.cpp"
    "./DriveStateMachine.cpp"
)
