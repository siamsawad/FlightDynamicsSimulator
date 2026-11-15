#pragma once

#include "FlightSim/Aircraft.hpp"

namespace flight {

// Results from one autopilot protection update.
struct EnvelopeStatus {

    bool lowAirspeed = false;
    bool stallRecovery = false;

    double minimumRecommendedAirspeedMetersPerSecond = 0.0;

    // Normalized elevator request, from -1.0 to +1.0.
    double appliedElevatorAxis = 0.0;
};

// Experimental protection for AUTOPILOT elevator commands.
//
// Manual elevator input is not restricted by this class.
class FlightEnvelopeProtection {
public:

    EnvelopeStatus update(
        const AircraftState& state,
        const AircraftParameters& aircraft,
        const Loads& loads,
        double requestedElevatorAxis,
        bool autopilotEngaged
    );

    // Clear protection state when restarting a flight.
    void reset();

private:

    bool lowAirspeedActive_ = false;
    bool stallRecoveryActive_ = false;
};

} // namespace flight