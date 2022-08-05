
#include "unit_conversions.hpp"

uint32_t radians_to_encoder_count(const double radians)
{
    constexpr uint16_t encoder_resolution = 8000U;
    constexpr double pi                   = 3.14159265358979323846;
    return (encoder_resolution * radians) / (2 * pi);
}

uint32_t calculate_velocity(const double rad_per_sec)
{
    constexpr uint16_t sample_rate_hz = 8000U;
    constexpr double sample_period    = 1.0 / sample_rate_hz;
    uint32_t encoder_count            = radians_to_encoder_count(rad_per_sec);
    return (encoder_count * sample_period) * ((uint32_t)1U << 16U);
}

uint32_t calculate_acceleration(const double rad_per_sec_per_sec)
{
    constexpr uint16_t sample_rate_hz = 8000U;
    constexpr double sample_period    = 1.0 / sample_rate_hz;
    uint32_t encoder_count            = radians_to_encoder_count(rad_per_sec_per_sec);
    return (encoder_count * sample_period * sample_period) * ((uint32_t)1U << 16U);
}
