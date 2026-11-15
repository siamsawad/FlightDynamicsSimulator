
#include "FlightSim/Autopilot6DOF.hpp"
#include "FlightSim/Controls6DOF.hpp"
#include "FlightSim/Dynamics6DOF.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {

bool near(
    double actual,
    double expected,
    double tolerance = 1e-8
) {

    return std::abs(actual - expected) <= tolerance;
}

bool check(
    bool condition,
    const char* description
) {

    if (!condition) {

        std::cerr
            << "FAIL: " << description << '\n';
    }

    return condition;
}

} // anonymous namespace

int main() {

    using namespace flight;
    using namespace flight::sixdof;

    // ========================================================
    // 1. INITIALIZE THE TEST AIRCRAFT
    // ========================================================

    State state;

    state.bodyToWorld = Quaternion::fromEuler(
        0.0,
        radians(1.75),
        0.0
    );

    Model model;
    Inputs controls;
    Wind wind;

    const AirData normal =
        calculateForcesMoments(
            state,
            model,
            controls,
            wind
        ).air;

    Autopilot6DOF autopilot;

    const Controls6DOF actuator;

    int checks = 0;

    // Count verified conditions.
    const auto verify = [&](
        bool condition,
        const char* message
    ) {

        ++checks;
        return check(condition, message);
    };

    // ========================================================
    // TEST 1: AUTOPILOT STARTS DISENGAGED
    // ========================================================

    if (!verify(
        !autopilot.isEngaged() &&
        near(
            autopilot.update(
                state, normal, model, kTimeStep
            ).pitch,
            0.0
        ),
        "Autopilot starts disengaged"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 2: CAPTURE ALTITUDE AND HEADING
    // ========================================================

    autopilot.engage(state);

    if (!verify(
        autopilot.isEngaged() &&
        near(
            autopilot.status().targetAltitudeMeters,
            1000.0
        ) &&
        near(
            autopilot.status().targetHeadingRadians,
            0.0
        ),
        "Engagement captures current flight conditions"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 3: CLIMB COMMAND
    // ========================================================

    autopilot.setTargetAltitudeMeters(1030.48);

    PilotAxes command =
        autopilot.update(
            state, normal, model, kTimeStep
        );

    if (!verify(
        command.pitch > 0.1 &&
        autopilot.status().altitudeErrorMeters > 30.0,
        "Positive altitude error requests climb"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 4: DESCENT COMMAND
    // ========================================================

    autopilot.setTargetAltitudeMeters(970.0);

    command = autopilot.update(
        state, normal, model, kTimeStep
    );

    if (!verify(
        command.pitch < 0.0 &&
        autopilot.status().altitudeErrorMeters < 0.0,
        "Negative altitude error requests descent"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 5: RIGHT HEADING COMMAND
    // ========================================================

    autopilot.engage(state);

    autopilot.setTargetHeadingRadians(
        radians(20.0)
    );

    command = autopilot.update(
        state, normal, model, kTimeStep
    );

    if (!verify(
        command.roll > 0.0 &&
        command.yaw > 0.0 &&
        autopilot.status().desiredBankRadians > 0.0,
        "Right turn requests positive bank and rudder"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 6: LEFT HEADING COMMAND
    // ========================================================

    autopilot.setTargetHeadingRadians(
        radians(-20.0)
    );

    command = autopilot.update(
        state, normal, model, kTimeStep
    );

    if (!verify(
        command.roll < 0.0 &&
        command.yaw < 0.0,
        "Left turn requests negative bank and rudder"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 7: SHORTEST HEADING CORRECTION
    // ========================================================

    // From 179 degrees to -179 degrees is only a
    // two-degree right turn, not a 358-degree left turn.
    State nearWrap = state;

    nearWrap.bodyToWorld =
        Quaternion::fromEuler(
            0.0,
            radians(1.75),
            radians(179.0)
        );

    autopilot.engage(nearWrap);

    autopilot.setTargetHeadingRadians(
        radians(-179.0)
    );

    const AirData wrapAir =
        calculateForcesMoments(
            nearWrap,
            model,
            controls,
            wind
        ).air;

    autopilot.update(
        nearWrap,
        wrapAir,
        model,
        kTimeStep
    );

    if (!verify(
        near(
            degrees(
                autopilot.status().headingErrorRadians
            ),
            2.0,
            1e-6
        ),
        "Heading wraps across 180 degrees"
    )) {
        return 1;
    }

    // ========================================================
    // TESTS 8-9: ALTITUDE TARGET LIMITS
    // ========================================================

    autopilot.setTargetAltitudeMeters(-100.0);

    if (!verify(
        near(
            autopilot.status().targetAltitudeMeters,
            0.0
        ),
        "Minimum altitude target"
    )) {
        return 1;
    }

    autopilot.setTargetAltitudeMeters(12000.0);

    if (!verify(
        near(
            autopilot.status().targetAltitudeMeters,
            11000.0
        ),
        "Maximum altitude target"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 10: LOW-AIRSPEED RESPONSE
    // ========================================================

    autopilot.engage(state);
    autopilot.adjustTargetAltitudeMeters(30.0);

    AirData slow = normal;

    slow.trueAirspeedMetersPerSecond = 25.0;

    command = autopilot.update(
        state,
        slow,
        model,
        kTimeStep
    );

    if (!verify(
        autopilot.status().lowAirspeed &&
        command.pitch <= -0.1,
        "Low airspeed reduces nose-up demand"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 11: POSITIVE HIGH-AOA RESPONSE
    // ========================================================

    AirData stall = normal;

    stall.angleOfAttackRadians =
        radians(17.0);

    command = autopilot.update(
        state,
        stall,
        model,
        kTimeStep
    );

    if (!verify(
        autopilot.status().highAngleOfAttack &&
        near(command.pitch, -0.8),
        "Positive high AoA requests nose down"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 12: NEGATIVE HIGH-AOA RESPONSE
    // ========================================================

    stall.angleOfAttackRadians =
        radians(-17.0);

    command = autopilot.update(
        state,
        stall,
        model,
        kTimeStep
    );

    if (!verify(
        autopilot.status().highAngleOfAttack &&
        near(command.pitch, 0.8),
        "Negative high AoA requests nose up"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 13: THROTTLE IS NOT AUTOPILOT-CONTROLLED
    // ========================================================

    if (!verify(
        near(command.throttle, 0.0),
        "Autopilot does not directly control throttle"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 14: INVALID TIMESTEP
    // ========================================================

    bool caught = false;

    try {

        autopilot.update(
            state,
            normal,
            model,
            0.0
        );

    } catch (const std::invalid_argument&) {

        caught = true;
    }

    if (!verify(
        caught,
        "Invalid timestep rejected"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 15: INVALID ALTITUDE TARGET
    // ========================================================

    caught = false;

    try {

        autopilot.setTargetAltitudeMeters(
            std::numeric_limits<double>::quiet_NaN()
        );

    } catch (const std::invalid_argument&) {

        caught = true;
    }

    if (!verify(
        caught,
        "Nonfinite altitude target rejected"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 16: DISENGAGEMENT
    // ========================================================

    autopilot.disengage();

    command = autopilot.update(
        state,
        normal,
        model,
        kTimeStep
    );

    if (!verify(
        !autopilot.isEngaged() &&
        near(command.pitch, 0.0) &&
        near(command.roll, 0.0),
        "Disengaged controller produces neutral commands"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 17: FULL CLOSED-LOOP FLIGHT
    // ========================================================

    // Start a new flight using the established 6-DOF model.
    State flight = state;

    Inputs applied;

    Autopilot6DOF controller;

    controller.engage(flight);

    // Request a 100-foot climb and a 20-degree turn.
    controller.adjustTargetAltitudeMeters(
        100.0 * kFeetToMeters
    );

    controller.adjustTargetHeadingRadians(
        radians(20.0)
    );

    double maximumBankDegrees = 0.0;
    double minimumSpeed = 1e9;

    // Run 90 simulated seconds at 120 Hz.
    for (int i = 0; i < 90 * 120; ++i) {

        const AirData air =
            calculateForcesMoments(
                flight,
                model,
                applied,
                wind
            ).air;

        const PilotAxes commandAxes =
            controller.update(
                flight,
                air,
                model,
                kTimeStep
            );

        actuator.update(
            applied,
            commandAxes,
            kTimeStep
        );

        step(
            flight,
            model,
            applied,
            wind,
            kTimeStep
        );

        const EulerAngles angles =
            toEuler(flight.bodyToWorld);

        maximumBankDegrees = std::max(
            maximumBankDegrees,
            std::abs(
                degrees(angles.rollRadians)
            )
        );

        minimumSpeed = std::min(
            minimumSpeed,
            air.trueAirspeedMetersPerSecond
        );
    }

    // --------------------------------------------------------
    // INSPECT FINAL ALTITUDE AND HEADING ERRORS
    // --------------------------------------------------------

    const EulerAngles finalAttitude =
        toEuler(flight.bodyToWorld);

    // Negative world Z means positive altitude.
    const double altitudeError =
        controller.status().targetAltitudeMeters +
        flight.positionWorldMeters.z;

    const double headingError = degrees(
        std::remainder(
            controller.status().targetHeadingRadians -
            finalAttitude.yawRadians,
            2.0 * kPi
        )
    );

    std::cout
        << "90-second combined maneuver:\n"
        << "Altitude error: "
        << altitudeError << " m\n"
        << "Heading error: "
        << headingError << " deg\n"
        << "Maximum bank: "
        << maximumBankDegrees << " deg\n"
        << "Minimum airspeed: "
        << minimumSpeed << " m/s\n";

    // Baseline acceptance criteria for this project.
    if (!verify(
        std::abs(altitudeError) < 4.0 &&
        std::abs(headingError) < 3.0 &&
        maximumBankDegrees < 30.0 &&
        minimumSpeed > 36.0,
        "Closed-loop tracking under baseline conditions"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 18: RE-ENGAGEMENT RESETS CONTROLLER
    // ========================================================

    controller.disengage();

    controller.engage(state);

    if (!verify(
        near(
            controller.status().targetAltitudeMeters,
            1000.0
        ) &&
        near(
            controller.status().targetHeadingRadians,
            0.0
        ),
        "Re-engagement resets controller state"
    )) {
        return 1;
    }

    std::cout
        << "PASS: All " << checks
        << " 6-DOF autopilot checks passed.\n";

    return 0;
}
