
#pragma once

#include "FlightSim/Aircraft.hpp"

namespace flight {

// Advance the aircraft state by one fixed time step.
// Uses semi-implicit Euler numerical integration.
void stepPhysics(
    AircraftState& state,
    const AircraftParameters& aircraft,
    const Loads& loads,
    double dt
);

} // namespace flight
