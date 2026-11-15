
#include "FlightSim/ControlSystem.hpp"

#include <cmath>
#include <iostream>

using namespace flight;

// Compare floating-point values using a small tolerance.
bool near(double actual, double expected) {
    return std::abs(actual - expected) < 1e-9;
}

// Print a failure message and return false.
bool check(bool condition, const char* message) {

    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }

    return true;
}

int main() {

    const ControlSystem controlSystem;

    // ========================================================
    // TEST 1: Elevator does not move instantaneously
    // ========================================================

    Controls controls;
    PilotInput input;

    // Request full nose-up elevator.
    input.elevatorAxis = 1.0;

    // Update for 0.1 second.
    controlSystem.update(controls, input, 0.1);

    // Expected movement:
    // 0.30 command units/s * 0.1 s = 0.03.
    if (!check(
        near(controls.elevator, 0.03),
        "Elevator rate limit"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 2: Elevator reaches target without overshooting
    // ========================================================

    // Three additional 0.1-second steps.
    for (int i = 0; i < 3; ++i) {
        controlSystem.update(controls, input, 0.1);
    }

    if (!check(
        near(controls.elevator, 0.12),
        "Elevator reaches target"
    )) {
        return 1;
    }

    // Another update should not exceed 0.12.
    controlSystem.update(controls, input, 0.1);

    if (!check(
        near(controls.elevator, 0.12),
        "Elevator maximum limit"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 3: Releasing elevator returns toward neutral
    // ========================================================

    input.elevatorAxis = 0.0;

    controlSystem.update(controls, input, 0.1);

    if (!check(
        near(controls.elevator, 0.09),
        "Elevator returns toward neutral"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 4: Nose-down input works
    // ========================================================

    controls = Controls{};
    input.elevatorAxis = -1.0;

    controlSystem.update(controls, input, 0.1);

    if (!check(
        near(controls.elevator, -0.03),
        "Nose-down elevator"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 5: Throttle increases correctly
    // ========================================================

    controls = Controls{};
    input = PilotInput{};

    input.throttleAxis = 1.0;

    // Initial throttle = 0.30.
    // Increase = 0.35 * 0.1 = 0.035.
    controlSystem.update(controls, input, 0.1);

    if (!check(
        near(controls.throttle, 0.335),
        "Throttle increase"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 6: Throttle stays between zero and one
    // ========================================================

    for (int i = 0; i < 100; ++i) {
        controlSystem.update(controls, input, 0.1);
    }

    if (!check(
        near(controls.throttle, 1.0),
        "Maximum throttle limit"
    )) {
        return 1;
    }

    input.throttleAxis = -1.0;

    for (int i = 0; i < 100; ++i) {
        controlSystem.update(controls, input, 0.1);
    }

    if (!check(
        near(controls.throttle, 0.0),
        "Minimum throttle limit"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 7: Fixed 120 Hz stepping reaches same target
    // ========================================================

    controls = Controls{};
    input = PilotInput{};
    input.elevatorAxis = 1.0;

    constexpr double dt = 1.0 / 120.0;

    // 48 updates = 0.4 seconds.
    for (int i = 0; i < 48; ++i) {
        controlSystem.update(controls, input, dt);
    }

    if (!check(
        near(controls.elevator, 0.12),
        "Fixed-timestep control response"
    )) {
        return 1;
    }

    std::cout
        << "PASS: All seven control tests passed.\n";

    return 0;
}
