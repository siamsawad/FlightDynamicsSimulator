#pragma once

#include "FlightSim/Aircraft.hpp"
#include "FlightSim/Environment.hpp"

#include <string>

namespace flight {

// ============================================================
// SIMULATION CONFIGURATION
// ============================================================

// Stores all settings selected by the user.
//
// The default values reproduce our existing starting
// conditions when wind = 0 and density multiplier = 1.
//
// AircraftParameters already contains the mass,
// wingspan, wing area, thrust, and aerodynamic coefficients.
struct SimulationConfig {

    // Starting altitude, displayed in feet.
    double initialAltitudeFeet =
        1000.0 / kFeetToMeters;

    // Starting true airspeed, displayed in knots.
    double initialAirspeedKnots =
        50.0 / kKnotsToMetersPerSecond;

    // Target used when altitude-hold autopilot is engaged.
    double autopilotTargetFeet =
        1000.0 / kFeetToMeters;

    // Existing aircraft parameters.
    AircraftParameters aircraft{};

    // New environmental conditions.
    Environment environment{};
};

// Reports whether a configuration is acceptable.
struct ConfigValidation {

    bool valid = false;

    // Explanation shown when validation fails.
    std::string message;
};

// Validate numerical ranges and physical geometry.
ConfigValidation validateSimulationConfig(
    const SimulationConfig& config
);

// Create a fresh aircraft state from selected settings.
AircraftState makeInitialAircraftState(
    const SimulationConfig& config
);

} // namespace flight