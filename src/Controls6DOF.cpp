
#include "FlightSim/Controls6DOF.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace flight::sixdof {

namespace {

// Move toward a target without exceeding the rate limit.
double approach(
    double current,
    double target,
    double change
) {

    return current + std::clamp(
        target - current,
        -change,
        change
    );
}

} // anonymous namespace

// ============================================================
// UPDATE AIRCRAFT CONTROL SURFACES
// ============================================================

void Controls6DOF::update(
    Inputs& controls,
    const PilotAxes& pilot,
    double dtSeconds
) const {

    // Reject invalid or excessively large time steps.
    if (
        !std::isfinite(dtSeconds)
        || dtSeconds <= 0.0
        || dtSeconds > 0.05
    ) {

        throw std::invalid_argument(
            "6-DOF control timestep must be in (0, 0.05] s"
        );
    }

    // Check that pilot input is numerically valid.
    const auto valid = [](double value) {
        return std::isfinite(value);
    };

    if (
        !valid(pilot.pitch)
        || !valid(pilot.roll)
        || !valid(pilot.yaw)
        || !valid(pilot.throttle)
    ) {

        throw std::invalid_argument(
            "Nonfinite pilot axis"
        );
    }

    // --------------------------------------------------------
    // 1. ELEVATOR
    // --------------------------------------------------------

    // Moves up to 2.5 normalized units per second.
    controls.elevator = approach(
        controls.elevator,
        std::clamp(pilot.pitch, -1.0, 1.0),
        2.5 * dtSeconds
    );

    // --------------------------------------------------------
    // 2. AILERONS
    // --------------------------------------------------------

    // Moves up to 2.0 normalized units per second.
    controls.aileron = approach(
        controls.aileron,
        std::clamp(pilot.roll, -1.0, 1.0),
        2.0 * dtSeconds
    );

    // --------------------------------------------------------
    // 3. RUDDER
    // --------------------------------------------------------

    controls.rudder = approach(
        controls.rudder,
        std::clamp(pilot.yaw, -1.0, 1.0),
        2.0 * dtSeconds
    );

    // --------------------------------------------------------
    // 4. THROTTLE
    // --------------------------------------------------------

    // Throttle behaves differently from control surfaces:
    //
    // Releasing a throttle key maintains current power.
    // Releasing a control-surface key returns it to neutral.
    controls.throttle = std::clamp(
        controls.throttle
        + std::clamp(
            pilot.throttle,
            -1.0,
            1.0
        ) * 0.35 * dtSeconds,
        0.0,
        1.0
    );
}

} // namespace flight::sixdof
