#pragma once

#include "FlightSim/Aircraft.hpp"
#include "FlightSim/Environment.hpp"

namespace flight {

// NEW: Calculate loads using custom environmental conditions.
Loads calculateLoads(
    const AircraftState& state,
    const AircraftParameters& aircraft,
    const Controls& controls,
    const Environment& environment
);

// COMPATIBILITY: Preserve our previous interface.
//
// All older code that calls calculateLoads with three
// arguments will automatically use standard conditions:
// no wind, density multiplier = 1.
inline Loads calculateLoads(
    const AircraftState& state,
    const AircraftParameters& aircraft,
    const Controls& controls
) {
    return calculateLoads(
        state,
        aircraft,
        controls,
        Environment{}
    );
}

} // namespace flight