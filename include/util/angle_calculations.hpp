#ifndef OSRM_UTIL_GUIDANCE_ANGLE_CALCULATIONS_HPP_
#define OSRM_UTIL_GUIDANCE_ANGLE_CALCULATIONS_HPP_

#include <algorithm>
#include <cmath>

namespace osrm
{
namespace util
{
namespace guidance
{

inline double angularDeviation(const double angle, const double from)
{
    const double deviation = std::abs(angle - from);
    return std::min(360 - deviation, deviation);
}

} // namespace guidance
} // namespace util
} // namespace osrm

#endif /* OSRM_UTIL_GUIDANCE_ANGLE_CALCULATIONS_HPP_ */
