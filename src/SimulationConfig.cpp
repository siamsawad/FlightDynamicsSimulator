#include "FlightSim/SimulationConfig.hpp"

#include <cmath>
#include <stdexcept>

namespace flight {

// ============================================================
// VALIDATE USER CONFIGURATION
// ============================================================

ConfigValidation validateSimulationConfig(
    const SimulationConfig& c
) {

    // Check that a number is finite and inside a range.
    //
    // std::isfinite rejects NaN and infinity, which
    // would otherwise cause numerical simulation problems.
    const auto inRange = [](
        double value,
        double minimum,
        double maximum
    ) {
        return std::isfinite(value)
            && value >= minimum
            && value <= maximum;
    };

    // --------------------------------------------------------
    // INITIAL FLIGHT CONDITIONS
    // --------------------------------------------------------

    if (!inRange(c.initialAltitudeFeet, 0.0, 30000.0)) {
        return {
            false,
            "Starting altitude must be 0 to 30,000 ft"
        };
    }

    if (!inRange(c.initialAirspeedKnots, 60.0, 220.0)) {
        return {
            false,
            "Starting airspeed must be 60 to 220 kt"
        };
    }

    if (!inRange(c.autopilotTargetFeet, 0.0, 30000.0)) {
        return {
            false,
            "Autopilot target must be 0 to 30,000 ft"
        };
    }

    // --------------------------------------------------------
    // AIRCRAFT PHYSICAL PROPERTIES
    // --------------------------------------------------------

    if (!inRange(c.aircraft.massKg, 500.0, 3000.0)) {
        return {
            false,
            "Aircraft mass must be 500 to 3,000 kg"
        };
    }

    if (!inRange(c.aircraft.wingSpanMeters, 7.0, 18.0)) {
        return {
            false,
            "Wingspan must be 7 to 18 m"
        };
    }

    if (!inRange(c.aircraft.wingAreaM2, 8.0, 35.0)) {
        return {
            false,
            "Wing area must be 8 to 35 m^2"
        };
    }

    if (!inRange(
        c.aircraft.maxThrustNewtons,
        500.0,
        12000.0
    )) {
        return {
            false,
            "Maximum thrust must be 500 to 12,000 N"
        };
    }

    // --------------------------------------------------------
    // ENVIRONMENT
    // --------------------------------------------------------

    if (!inRange(
        c.environment.headwindKnots,
        -45.0,
        45.0
    )) {
        return {
            false,
            "Wind must be between -45 and +45 kt"
        };
    }

    if (!inRange(
        c.environment.airDensityMultiplier,
        0.70,
        1.30
    )) {
        return {
            false,
            "Air density multiplier must be 0.70 to 1.30"
        };
    }

    // --------------------------------------------------------
    // AIRCRAFT GEOMETRY CHECK
    // --------------------------------------------------------

    // Wing aspect ratio:
    //
    // AR = wingspan^2 / wing area
    //
    // This helps reject extreme combinations even when
    // wingspan and wing area individually look reasonable.
    const double aspectRatio =
        c.aircraft.wingSpanMeters *
        c.aircraft.wingSpanMeters /
        c.aircraft.wingAreaM2;

    if (aspectRatio < 3.0 || aspectRatio > 20.0) {
        return {
            false,
            "Wingspan and area require aspect ratio 3 to 20"
        };
    }

    return {
        true,
        "Configuration is valid"
    };
}

// ============================================================
// CREATE INITIAL AIRCRAFT STATE
// ============================================================

AircraftState makeInitialAircraftState(
    const SimulationConfig& config
) {

    // Never initialize the physics engine with an invalid
    // or unsupported set of user inputs.
    const ConfigValidation validation =
        validateSimulationConfig(config);

    if (!validation.valid) {
        throw std::invalid_argument(validation.message);
    }

    // Begin with our established aircraft-state defaults.
    AircraftState state;

    // Convert user-selected feet to internal meters.
    state.positionMeters.z =
        config.initialAltitudeFeet * kFeetToMeters;

    // Convert starting TRUE AIRSPEED into ground velocity.
    //
    // Our world convention is +X forward.
    // A positive headwind moves air toward -X.
    //
    // Therefore:
    //
    // ground speed = airspeed - headwind speed
    //
    // This ensures the configured airspeed is correct
    // immediately when the simulation starts.
    state.velocityMetersPerSecond.x =
        (
            config.initialAirspeedKnots
            - config.environment.headwindKnots
        ) * kKnotsToMetersPerSecond;

    // Initial vertical velocity is zero.
    state.velocityMetersPerSecond.z = 0.0;

    // Preserve the established starting pitch angle.
    state.pitchRadians = radians(1.75);

    // Start without any pitch rotation.
    state.pitchRateRadiansPerSecond = 0.0;

    return state;
}

} // namespace flight