
#include "FlightSim/ControlSystem.hpp"

#include <algorithm> // clamp, min
#include <cmath>      // isfinite
#include <stdexcept>  // invalid_argument

namespace flight {

namespace {

// Move a numerical value toward a target while limiting
// how far it can change during one simulation step.
//
// Example:
// current = 0.00
// target = 0.12
// maxChange = 0.03
//
// Result = 0.03, not 0.12.
double moveTowards(
    double current,
    double target,
    double maxChange
) {

    // Prevent overshooting the target.
    if (target > current) {
        return current + std::min(
            target - current,
            maxChange
        );
    }

    if (target < current) {
        return current - std::min(
            current - target,
            maxChange
        );
    }

    // Already at the target.
    return current;
}

} // anonymous namespace

// ============================================================
// CONTROL SYSTEM UPDATE
// ============================================================

void ControlSystem::update(
    Controls& controls,
    const PilotInput& input,
    double dtSeconds
) const {

    // Numerical integration requires a valid time step.
    if (!std::isfinite(dtSeconds) || dtSeconds <= 0.0) {
        throw std::invalid_argument(
            "Control timestep must be positive and finite"
        );
    }

    // --------------------------------------------------------
    // 1. CALCULATE DESIRED ELEVATOR COMMAND
    // --------------------------------------------------------

    // Limit pilot input to the range [-1, +1].
    const double elevatorAxis =
        std::clamp(input.elevatorAxis, -1.0, 1.0);

    // Convert pilot request into our model's elevator range.
    const double targetElevator =
        elevatorAxis * kMaxElevatorCommand;

    // --------------------------------------------------------
    // 2. CALCULATE ELEVATOR MOVEMENT LIMIT
    // --------------------------------------------------------

    // Maximum amount the elevator command can change
    // during this single physics update.
    const double maximumChange =
        kElevatorRate * dtSeconds;

    // Gradually move the actual elevator command toward
    // the requested value.
    //
    // Releasing the key requests a neutral elevator,
    // so the command will also return to zero gradually.
    controls.elevator = moveTowards(
        controls.elevator,
        targetElevator,
        maximumChange
    );

    // --------------------------------------------------------
    // 3. UPDATE THROTTLE
    // --------------------------------------------------------

    // Throttle input is also limited to [-1, +1].
    const double throttleAxis =
        std::clamp(input.throttleAxis, -1.0, 1.0);

    // W/S change the throttle setting progressively.
    // This preserves our Version 3 throttle behavior,
    // but moves the update into the fixed physics loop.
    controls.throttle +=
        throttleAxis * kThrottleRate * dtSeconds;

    // Throttle cannot exceed 100% or fall below 0%.
    controls.throttle = std::clamp(
        controls.throttle,
        0.0,
        1.0
    );
}

} // namespace flight
