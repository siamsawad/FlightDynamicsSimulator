
#pragma once

#include "FlightSim/Aircraft.hpp"

namespace flight {

// What the pilot is currently requesting.
//
// These values describe keyboard/stick INPUT,
// not the aircraft's actual control surface position.
//
// elevatorAxis:
//   +1.0 = request nose-up elevator
//   -1.0 = request nose-down elevator
//    0.0 = neutral
//
// throttleAxis:
//   +1.0 = increase throttle
//   -1.0 = decrease throttle
//    0.0 = leave throttle unchanged
struct PilotInput {
    double elevatorAxis = 0.0;
    double throttleAxis = 0.0;
};

// Simulates how pilot input becomes actual control commands.
//
// Aircraft.hpp already contains our Controls struct.
// We will keep using that struct to store the ACTUAL
// throttle and elevator values applied to aerodynamics.
//
// Keeping input and actuator response separate will make
// it easier to introduce a PID autopilot or future
// roll/yaw controls without rewriting the physics engine.
class ControlSystem {
public:

    // Update the actual controls by ONE physics time step.
    //
    // controls: Modified actual control values.
    // input: Pilot's requested controls.
    // dtSeconds: Fixed physics time step.
    void update(
        Controls& controls,
        const PilotInput& input,
        double dtSeconds
    ) const;

private:

    // Preserve our previous maximum elevator command.
    // These are normalized model commands, not angles.
    static constexpr double kMaxElevatorCommand = 0.12;

    // Maximum elevator command change per second.
    static constexpr double kElevatorRate = 0.30;

    // Throttle change per second while W or S is pressed.
    static constexpr double kThrottleRate = 0.35;
};

} // namespace flight
