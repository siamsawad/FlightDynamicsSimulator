
#include "FlightSim/Aerodynamics.hpp"
#include "FlightSim/Autopilot.hpp"
#include "FlightSim/ControlSystem.hpp"
#include "FlightSim/PhysicsEngine.hpp"

#include <cmath>
#include <iostream>

namespace {

// Compare floating-point values with a tolerance.
bool near(
    double actual,
    double expected,
    double tolerance
) {
    return std::abs(actual - expected) <= tolerance;
}

// Display a useful message when a test fails.
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

    AircraftState state;
    Autopilot autopilot;

    // ========================================================
    // TEST 1: AUTOPILOT STARTS DISENGAGED
    // ========================================================

    if (!check(
        !autopilot.isEngaged() &&
        autopilot.update(state, kTimeStep) == 0.0,
        "Autopilot starts disengaged"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 2: ENGAGEMENT CAPTURES CURRENT ALTITUDE
    // ========================================================

    autopilot.engage(state);

    if (!check(
        autopilot.isEngaged() &&
        near(
            autopilot.targetAltitudeMeters(),
            1000.0,
            1e-10
        ),
        "Capture current altitude"
    )) {
        return 1;
    }

    // At the captured altitude with zero vertical speed,
    // the controller should start near zero output.
    if (!check(
        std::abs(
            autopilot.update(state, kTimeStep)
        ) < 0.001,
        "Neutral initial altitude-hold command"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 3: BELOW TARGET -> NOSE-UP REQUEST
    // ========================================================

    autopilot.setTargetAltitudeMeters(1050.0);

    if (!check(
        autopilot.update(state, kTimeStep) > 0.5,
        "Positive altitude error"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 4: ABOVE TARGET -> NOSE-DOWN REQUEST
    // ========================================================

    autopilot.setTargetAltitudeMeters(950.0);

    if (!check(
        autopilot.update(state, kTimeStep) < -0.5,
        "Negative altitude error"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 5: DERIVATIVE DAMPING
    // ========================================================

    // Re-engage to reset the target and integral state.
    autopilot.engage(state);

    // Simulate an aircraft climbing at 3 m/s.
    state.velocityMetersPerSecond.z = 3.0;

    // At target altitude, the autopilot should respond
    // with a nose-down request to slow the climb.
    if (!check(
        autopilot.update(state, kTimeStep) < 0.0,
        "Vertical-speed damping"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 6: DISENGAGEMENT
    // ========================================================

    autopilot.disengage();

    if (!check(
        !autopilot.isEngaged() &&
        autopilot.update(state, kTimeStep) == 0.0,
        "Manual disengagement"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 7: TARGET ALTITUDE LIMITS
    // ========================================================

    autopilot.setTargetAltitudeMeters(-10.0);

    if (!check(
        near(autopilot.targetAltitudeMeters(), 0.0, 1e-10),
        "Minimum target altitude"
    )) {
        return 1;
    }

    autopilot.setTargetAltitudeMeters(15000.0);

    if (!check(
        near(autopilot.targetAltitudeMeters(), 11000.0, 1e-10),
        "Maximum target altitude"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 8: OUTPUT LIMITS AND INTEGRATOR RESET
    // ========================================================

    state = AircraftState{};

    autopilot.engage(state);
    autopilot.adjustTargetAltitudeMeters(50.0);

    if (!check(
        near(autopilot.targetAltitudeMeters(), 1050.0, 1e-10),
        "Target altitude adjustment"
    )) {
        return 1;
    }

    // Repeatedly run the controller with an unchanged state.
    // Check that the output never exceeds actuator limits.
    for (int i = 0; i < 5000; ++i) {

        const double output =
            autopilot.update(state, kTimeStep);

        if (!check(
            output >= -1.0 && output <= 1.0,
            "Elevator request saturation"
        )) {
            return 1;
        }
    }

    // Re-engagement should clear accumulated altitude error.
    autopilot.disengage();
    autopilot.engage(state);

    if (!check(
        std::abs(
            autopilot.update(state, kTimeStep)
        ) < 0.001,
        "Reset integral state on re-engagement"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 9: COMPLETE CLOSED-LOOP FLIGHT SIMULATION
    // ========================================================

    // This is the most important integration test.
    //
    // It runs the actual aircraft dynamics, ControlSystem,
    // and autopilot together without opening an SFML window.
    //
    // The aircraft flies for 90 simulated seconds.
    //
    // At 5 seconds, we command a 50-meter climb.
    //
    // Expected behavior:
    // - Aircraft climbs toward target.
    // - Vertical speed gradually decreases.
    // - Final altitude is reasonably close to target.

    AircraftState flightState;
    const AircraftParameters aircraft;
    Controls controls;

    const ControlSystem controlSystem;

    Autopilot flightAutopilot;
    flightAutopilot.engage(flightState);

    constexpr int totalSteps = 90 * 120;

    for (int step = 0; step < totalSteps; ++step) {

        // Command climb after five simulated seconds.
        if (step == 5 * 120) {
            flightAutopilot.adjustTargetAltitudeMeters(50.0);
        }

        // STEP 1: Autopilot calculates elevator request.
        PilotInput input;

        input.elevatorAxis =
            flightAutopilot.update(
                flightState,
                kTimeStep
            );

        // STEP 2: Apply actuator rate limits.
        controlSystem.update(
            controls,
            input,
            kTimeStep
        );

        // STEP 3: Calculate aircraft forces and moment.
        const Loads loads =
            calculateLoads(
                flightState,
                aircraft,
                controls
            );

        // STEP 4: Update the aircraft state.
        stepPhysics(
            flightState,
            aircraft,
            loads,
            kTimeStep
        );
    }

    // Inspect the final aircraft state.
    const double finalAltitude =
        flightState.positionMeters.z;

    const double finalVerticalSpeed =
        flightState.velocityMetersPerSecond.z;

    std::cout
        << "Closed-loop final altitude: "
        << finalAltitude << " m\n";

    std::cout
        << "Closed-loop vertical speed: "
        << finalVerticalSpeed << " m/s\n";

    // Allow a 4-meter altitude error for this initial test.
    // This is a project test tolerance, not an aviation
    // certification or safety performance requirement.
    if (!check(
        near(finalAltitude, 1050.0, 4.0),
        "Closed-loop altitude tracking"
    )) {
        return 1;
    }

    // Check that the climb/descent rate has become small.
    if (!check(
        std::abs(finalVerticalSpeed) < 0.35,
        "Closed-loop vertical-speed settling"
    )) {
        return 1;
    }

    std::cout
        << "PASS: All nine autopilot tests passed.\n";

    return 0;
}
