
#include "FlightSim/Aerodynamics.hpp"
#include "FlightSim/FlightEnvelope.hpp"
#include "FlightSim/GroundContact.hpp"
#include "FlightSim/PhysicsEngine.hpp"

#include <cmath>
#include <iostream>

namespace {

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

    AircraftParameters aircraft;
    AircraftState state;
    Controls controls;

    FlightEnvelopeProtection protection;

    // ========================================================
    // TEST 1: NORMAL CRUISE
    // ========================================================

    Loads loads =
        calculateLoads(state, aircraft, controls);

    auto status = protection.update(
        state,
        aircraft,
        loads,
        0.4,
        true
    );

    if (!check(
        !status.lowAirspeed &&
        !status.stallRecovery &&
        std::abs(status.appliedElevatorAxis - 0.4) < 1e-12,
        "Normal cruise leaves elevator request unchanged"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 2: LOW-AIRSPEED PROTECTION
    // ========================================================

    state.velocityMetersPerSecond.x = 25.0;

    loads =
        calculateLoads(state, aircraft, controls);

    status = protection.update(
        state,
        aircraft,
        loads,
        1.0,
        true
    );

    if (!check(
        status.lowAirspeed &&
        status.appliedElevatorAxis <= 0.0,
        "Low-speed elevator protection"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 3: LOW-SPEED HYSTERESIS
    // ========================================================

    // Raise speed slightly above the entry threshold.
    // Protection should remain active.
    state.velocityMetersPerSecond.x =
        status.minimumRecommendedAirspeedMetersPerSecond
        * 1.04;

    loads =
        calculateLoads(state, aircraft, controls);

    status = protection.update(
        state,
        aircraft,
        loads,
        0.8,
        true
    );

    if (!check(
        status.lowAirspeed,
        "Low-speed protection hysteresis"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 4: LOW-SPEED PROTECTION RELEASE
    // ========================================================

    state.velocityMetersPerSecond.x =
        status.minimumRecommendedAirspeedMetersPerSecond
        * 1.12;

    loads =
        calculateLoads(state, aircraft, controls);

    status = protection.update(
        state,
        aircraft,
        loads,
        0.6,
        true
    );

    if (!check(
        !status.lowAirspeed &&
        status.appliedElevatorAxis > 0.0,
        "Low-speed protection release"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 5: POSITIVE ANGLE-OF-ATTACK STALL RECOVERY
    // ========================================================

    state.velocityMetersPerSecond.x = 50.0;
    state.pitchRadians = radians(20.0);

    loads =
        calculateLoads(state, aircraft, controls);

    status = protection.update(
        state,
        aircraft,
        loads,
        +0.5,
        true
    );

    if (!check(
        status.stallRecovery &&
        status.appliedElevatorAxis == -1.0,
        "Positive-AoA stall recovery"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 6: STALL RECOVERY HYSTERESIS
    // ========================================================

    state.pitchRadians = radians(12.0);

    loads =
        calculateLoads(state, aircraft, controls);

    status = protection.update(
        state,
        aircraft,
        loads,
        +0.5,
        true
    );

    if (!check(
        status.stallRecovery,
        "Stall recovery hysteresis"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 7: RECOVERY MODE RELEASE
    // ========================================================

    state.pitchRadians = radians(9.0);

    loads =
        calculateLoads(state, aircraft, controls);

    status = protection.update(
        state,
        aircraft,
        loads,
        +0.5,
        true
    );

    if (!check(
        !status.stallRecovery,
        "Recovery deactivates"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 8: NEGATIVE ANGLE-OF-ATTACK RECOVERY
    // ========================================================

    state.pitchRadians = radians(-20.0);

    loads =
        calculateLoads(state, aircraft, controls);

    status = protection.update(
        state,
        aircraft,
        loads,
        -0.5,
        true
    );

    if (!check(
        status.stallRecovery &&
        status.appliedElevatorAxis == +1.0,
        "Negative-AoA stall recovery"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 9: MANUAL CONTROL BYPASSES PROTECTION
    // ========================================================

    status = protection.update(
        state,
        aircraft,
        loads,
        -0.5,
        false
    );

    if (!check(
        !status.stallRecovery &&
        !status.lowAirspeed &&
        status.appliedElevatorAxis == -0.5,
        "Manual command bypasses protection"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 10: RUNWAY COLLISION CONSTRAINT
    // ========================================================

    GroundContact ground;

    AircraftState landing;

    // Aircraft approaches the runway from 2 cm above it.
    landing.positionMeters.z = 0.02;

    landing.velocityMetersPerSecond.x = 10.0;
    landing.velocityMetersPerSecond.z = -5.0;

    // Create a downward net force including gravity.
    Loads downward;

    downward.netForceNewtons.z =
        -aircraft.massKg * kGravity;

    const double initialForward =
        landing.velocityMetersPerSecond.x;

    // First, let our original physics engine update.
    stepPhysics(
        landing,
        aircraft,
        downward,
        0.1
    );

    // Then resolve contact with the flat runway.
    auto contact = ground.resolve(
        landing,
        aircraft,
        downward,
        0.1
    );

    if (!check(
        contact.onGround &&
        contact.touchedDown &&
        landing.positionMeters.z == 0.0 &&
        landing.velocityMetersPerSecond.z == 0.0,
        "Runway collision constraint"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 11: HARD LANDING DETECTION
    // ========================================================

    if (!check(
        contact.hardLanding &&
        contact.impactSpeedMetersPerSecond > 3.0,
        "Hard-landing classification"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 12: ROLLING RESISTANCE
    // ========================================================

    if (!check(
        landing.velocityMetersPerSecond.x >= 0.0 &&
        landing.velocityMetersPerSecond.x < initialForward &&
        contact.rollingResistanceNewtons > 0.0,
        "Rolling resistance"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 13: LIFTOFF
    // ========================================================

    // Start from the runway with zero vertical velocity.
    landing.positionMeters.z = 0.0;
    landing.velocityMetersPerSecond.z = 0.0;

    // Create a net force that accelerates upward.
    Loads upward;
    upward.netForceNewtons.z = 20000.0;

    stepPhysics(
        landing,
        aircraft,
        upward,
        0.1
    );

    contact = ground.resolve(
        landing,
        aircraft,
        upward,
        0.1
    );

    if (!check(
        !contact.onGround &&
        landing.positionMeters.z > 0.0,
        "Lift-off when net force is upward"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 14: GROUND CONTACT RESET
    // ========================================================

    ground.reset();

    if (!check(
        landing.positionMeters.z > 0.0,
        "Ground contact reset"
    )) {
        return 1;
    }

    std::cout
        << "PASS: All 14 flight-safety checks passed.\n";

    return 0;
}
