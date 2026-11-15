
#pragma once

#include "FlightSim/Dynamics6DOF.hpp"

namespace flight::sixdof {

// Pilot commands: -1 to +1 for surfaces,
// -1 to +1 for throttle adjustment.
struct PilotAxes {

    // Positive = nose up.
    double pitch = 0.0;

    // Positive = bank right.
    double roll = 0.0;

    // Positive = yaw right.
    double yaw = 0.0;

    // Positive = increase engine power.
    double throttle = 0.0;
};

// Rate-limited actuators for the 6-DOF flight model.
class Controls6DOF {
public:

    void update(
        Inputs& controls,
        const PilotAxes& pilot,
        double dtSeconds
    ) const;
};

} // namespace flight::sixdof
