
#include "FlightSim/Controls6DOF.hpp"
#include "FlightSim/Projection3D.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace flight;
using namespace flight::sixdof;

namespace {

// Floating-point comparison.
bool near(
    double a,
    double b,
    double tolerance = 1e-9
) {

    return std::abs(a - b) < tolerance;
}

// Test-condition helper.
bool check(
    bool ok,
    const char* message
) {

    if (!ok) {
        std::cerr << "FAIL: " << message << '\n';
    }

    return ok;
}

} // anonymous namespace

int main() {

    Controls6DOF system;

    Inputs controls;
    PilotAxes pilot;

    // ========================================================
    // TEST 1: INITIAL CONTROL STATE
    // ========================================================

    if (!check(
        near(controls.throttle, 0.30)
        && near(controls.elevator, 0.0),
        "Initial neutral controls"
    )) {
        return 1;
    }

    // ========================================================
    // TESTS 2-5: RATE LIMITS
    // ========================================================

    pilot = {1.0, 1.0, 1.0, 1.0};

    // One control update of 0.05 seconds.
    system.update(
        controls,
        pilot,
        0.05
    );

    if (!check(
        near(controls.elevator, 0.125),
        "Elevator rate limit"
    )) {
        return 1;
    }

    if (!check(
        near(controls.aileron, 0.10),
        "Aileron rate limit"
    )) {
        return 1;
    }

    if (!check(
        near(controls.rudder, 0.10),
        "Rudder rate limit"
    )) {
        return 1;
    }

    if (!check(
        near(controls.throttle, 0.3175),
        "Throttle rate limit"
    )) {
        return 1;
    }

    // ========================================================
    // TESTS 6-7: SATURATION LIMITS
    // ========================================================

    // Continue holding maximum input.
    for (int i = 0; i < 100; ++i) {

        system.update(
            controls,
            pilot,
            0.05
        );
    }

    if (!check(
        near(controls.elevator, 1.0)
        && near(controls.aileron, 1.0)
        && near(controls.rudder, 1.0),
        "Control saturation"
    )) {
        return 1;
    }

    if (!check(
        near(controls.throttle, 1.0),
        "Throttle maximum"
    )) {
        return 1;
    }

    // ========================================================
    // TESTS 8-9: RELEASING CONTROLS
    // ========================================================

    // Return pilot input to neutral.
    pilot = PilotAxes{};

    system.update(
        controls,
        pilot,
        0.05
    );

    // Surfaces should move toward neutral.
    if (!check(
        near(controls.elevator, 0.875)
        && near(controls.aileron, 0.90)
        && near(controls.rudder, 0.90),
        "Surfaces return toward neutral"
    )) {
        return 1;
    }

    // Throttle should remain at the last selected value.
    if (!check(
        near(controls.throttle, 1.0),
        "Throttle holds when released"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 10: MINIMUM THROTTLE
    // ========================================================

    pilot.throttle = -1.0;

    for (int i = 0; i < 100; ++i) {

        system.update(
            controls,
            pilot,
            0.05
        );
    }

    if (!check(
        near(controls.throttle, 0.0),
        "Throttle minimum"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 11: INVALID TIMESTEP
    // ========================================================

    bool caught = false;

    try {

        system.update(
            controls,
            pilot,
            -0.01
        );

    } catch (const std::invalid_argument&) {

        caught = true;
    }

    if (!check(
        caught,
        "Invalid timestep rejected"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 12: CHASE CAMERA CENTERING
    // ========================================================

    State aircraft;

    const auto camera = makeChaseCamera(
        aircraft,
        1280.0,
        720.0
    );

    const auto projected = projectPoint(
        camera,
        aircraft.positionWorldMeters
    );

    if (!check(
        projected.inFront
        && std::abs(projected.xPixels - 640.0) < 1.0,
        "Chase camera centers aircraft horizontally"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 13: LEFT / RIGHT PROJECTION
    // ========================================================

    const auto right = projectPoint(
        camera,
        aircraft.positionWorldMeters
        + camera.rightWorld * 3.0
    );

    const auto left = projectPoint(
        camera,
        aircraft.positionWorldMeters
        - camera.rightWorld * 3.0
    );

    if (!check(
        left.inFront
        && right.inFront
        && left.xPixels < projected.xPixels
        && right.xPixels > projected.xPixels,
        "Perspective projects left/right correctly"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 14: NEAR-PLANE CLIPPING
    // ========================================================

    // Create a point behind the camera.
    const auto behind = projectPoint(
        camera,
        camera.positionWorld
        - camera.forwardWorld * 3.0
    );

    if (!check(
        !behind.inFront,
        "Behind-camera points clipped"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 15: FULL 6-DOF BANKING RESPONSE
    // ========================================================

    // This is a real integration test using the existing
    // six-degree-of-freedom physics engine.
    State bank = aircraft;

    Model model;

    pilot = PilotAxes{};
    pilot.roll = 1.0;

    controls = Inputs{};

    Wind wind;

    // Run two simulated seconds at 120 Hz.
    for (int i = 0; i < 240; ++i) {

        system.update(
            controls,
            pilot,
            kTimeStep
        );

        step(
            bank,
            model,
            controls,
            wind,
            kTimeStep
        );
    }

    const EulerAngles angles =
        toEuler(bank.bodyToWorld);

    if (!check(
        std::abs(angles.rollRadians) > 0.01,
        "Aileron commands bank in real 6-DOF engine"
    )) {
        return 1;
    }

    std::cout
        << "PASS: All 15 6-DOF control/camera "
        << "checks passed.\n";

    return 0;
}
