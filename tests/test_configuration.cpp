
#include "FlightSim/Aerodynamics.hpp"
#include "FlightSim/SimulationConfig.hpp"

#include <cmath>
#include <iostream>
#include <limits>

namespace {

// Compare floating-point values with a tolerance.
bool near(
    double actual,
    double expected,
    double tolerance = 1e-8
) {
    return std::abs(actual - expected) < tolerance;
}

// Print a failure message if a test fails.
bool check(bool condition, const char* message) {

    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }

    return true;
}

} // namespace

int main() {

    using namespace flight;

    // Default aircraft and environment.
    SimulationConfig config;

    // ========================================================
    // TEST 1: DEFAULT CONFIGURATION IS VALID
    // ========================================================

    if (!check(
        validateSimulationConfig(config).valid,
        "Default configuration validation"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 2: DEFAULT INITIAL STATE IS PRESERVED
    // ========================================================

    // Version 7 starts at 1000 m and 50 m/s.
    // Our new configuration system should preserve this.
    AircraftState state =
        makeInitialAircraftState(config);

    if (!check(
        near(state.positionMeters.z, 1000.0) &&
        near(state.velocityMetersPerSecond.x, 50.0),
        "Default starting conditions"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 3: PREVIOUS AERODYNAMIC API STILL WORKS
    // ========================================================

    // Compare the original three-argument call against
    // the new four-argument call with default conditions.
    const Controls controls;

    const Loads original =
        calculateLoads(
            state,
            config.aircraft,
            controls
        );

    const Loads updated =
        calculateLoads(
            state,
            config.aircraft,
            controls,
            Environment{}
        );

    if (!check(
        near(
            original.airspeedMetersPerSecond,
            updated.airspeedMetersPerSecond
        ) &&
        near(
            original.liftNewtons,
            updated.liftNewtons
        ) &&
        near(
            original.dragNewtons,
            updated.dragNewtons
        ),
        "No-wind aerodynamic compatibility"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 4: HEADWIND AND INITIAL AIRSPEED
    // ========================================================

    // The user selects TRUE airspeed.
    //
    // A 20-knot headwind should reduce ground speed,
    // but the initial airspeed should match the setting.
    SimulationConfig windy = config;

    windy.environment.headwindKnots = 20.0;

    const AircraftState windyState =
        makeInitialAircraftState(windy);

    const Loads windyLoads =
        calculateLoads(
            windyState,
            windy.aircraft,
            controls,
            windy.environment
        );

    const double requestedAirspeed =
        windy.initialAirspeedKnots *
        kKnotsToMetersPerSecond;

    if (!check(
        near(
            windyLoads.airspeedMetersPerSecond,
            requestedAirspeed
        ) &&
        windyState.velocityMetersPerSecond.x <
            state.velocityMetersPerSecond.x,
        "Initial airspeed with headwind"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 5: HEADWIND INCREASES AIRSPEED
    // ========================================================

    // Keep the original ground velocity fixed.
    //
    // Introduce a headwind and confirm that the
    // resulting air-relative speed becomes greater.
    const Loads headwindLoads =
        calculateLoads(
            state,
            config.aircraft,
            controls,
            windy.environment
        );

    if (!check(
        headwindLoads.airspeedMetersPerSecond >
            original.airspeedMetersPerSecond,
        "Headwind increases air-relative speed"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 6: TAILWIND REDUCES AIRSPEED
    // ========================================================

    windy.environment.headwindKnots = -20.0;

    const Loads tailwindLoads =
        calculateLoads(
            state,
            config.aircraft,
            controls,
            windy.environment
        );

    if (!check(
        tailwindLoads.airspeedMetersPerSecond <
            original.airspeedMetersPerSecond,
        "Tailwind decreases air-relative speed"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 7: AIR-DENSITY MULTIPLIER
    // ========================================================

    // At fixed speed and angle of attack, changing
    // density by a factor of 1.2 should scale lift
    // by the same factor.
    windy.environment.headwindKnots = 0.0;

    windy.environment.airDensityMultiplier = 1.20;

    const Loads denseAir =
        calculateLoads(
            state,
            config.aircraft,
            controls,
            windy.environment
        );

    if (!check(
        near(
            denseAir.liftNewtons / original.liftNewtons,
            1.20
        ),
        "Air density affects lift"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 8: INVALID MASS IS REJECTED
    // ========================================================

    SimulationConfig invalid = config;

    invalid.aircraft.massKg = 0.0;

    if (!check(
        !validateSimulationConfig(invalid).valid,
        "Reject zero aircraft mass"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 9: INVALID WING GEOMETRY IS REJECTED
    // ========================================================

    invalid = config;

    invalid.aircraft.wingSpanMeters = 7.0;
    invalid.aircraft.wingAreaM2 = 35.0;

    // This produces an aspect ratio below our limit.
    if (!check(
        !validateSimulationConfig(invalid).valid,
        "Reject invalid wing aspect ratio"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 10: NON-FINITE INPUT IS REJECTED
    // ========================================================

    invalid = config;

    invalid.environment.airDensityMultiplier =
        std::numeric_limits<double>::quiet_NaN();

    if (!check(
        !validateSimulationConfig(invalid).valid,
        "Reject NaN input"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 11: CUSTOM ALTITUDE INITIALIZATION
    // ========================================================

    SimulationConfig custom = config;

    custom.initialAltitudeFeet = 5000.0;

    const AircraftState customState =
        makeInitialAircraftState(custom);

    // 5000 ft * 0.3048 = 1524 meters.
    if (!check(
        near(
            customState.positionMeters.z,
            1524.0
        ),
        "Custom altitude conversion"
    )) {
        return 1;
    }

    std::cout
        << "PASS: All 11 configuration tests passed.\n";

    return 0;
}
