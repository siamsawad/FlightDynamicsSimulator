#pragma once

namespace flight {

// Environmental settings used by the aerodynamic model.
//
// Our current simulator is longitudinal (2D).
// Therefore, we model only wind along the X axis.
//
// Positive headwind: air moves against aircraft travel.
// Negative headwind: tailwind helps ground-relative travel.
struct Environment {

    // Wind speed in knots.
    double headwindKnots = 0.0;

    // 1.0 = standard atmosphere density.
    // 1.1 = 10% denser than standard.
    // 0.9 = 10% less dense than standard.
    double airDensityMultiplier = 1.0;
};

// Unit conversions used by the configuration system.

// One nautical mile = 1852 meters.
// One hour = 3600 seconds.
inline constexpr double kKnotsToMetersPerSecond =
    1852.0 / 3600.0;

// One foot = 0.3048 meters.
inline constexpr double kFeetToMeters = 0.3048;

} // namespace flight