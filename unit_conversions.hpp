#pragma once

#include <stdint.h>

uint32_t radians_to_encoder_count(double radians);

uint32_t calculate_velocity(double rad_per_sec);

uint32_t calculate_acceleration(double rad_per_sec_per_sec);
