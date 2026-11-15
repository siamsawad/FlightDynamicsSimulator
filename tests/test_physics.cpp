
#include "FlightSim/Aerodynamics.hpp"
#include "FlightSim/PhysicsEngine.hpp"

#include <cmath>
#include <iostream>

namespace {

// Compare floating-point values using a tolerance.
//
// Exact floating-point equality is often inappropriate
// because calculations involve numerical rounding.
bool near(
    double actual,
    double expected,
    double tolerance
) {
    return std::abs(actual - expected) <= tolerance;
}

// Check a test condition.
// Print an error if the condition fails.
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

    const AircraftParameters aircraft;

    // ========================================================
    // TEST 1: NUMERICAL INTEGRATION
    // ========================================================

    // Verify that known forces and moments produce the
    // expected velocity, position and pitch changes.
    {
        AircraftState state;
        Loads net;

        net.netForceNewtons.x = 1000.0;
        net.netForceNewtons.z = 2000.0;
        net.pitchMomentNoseUpNm = 2500.0;

        stepPhysics(state, aircraft, net, 0.1);

        if (!check(
            near(state.velocityMetersPerSecond.x, 50.1, 1e-10) &&
            near(state.velocityMetersPerSecond.z, 0.2, 1e-10) &&
            near(state.positionMeters.x, 5.01, 1e-10) &&
            near(state.positionMeters.z, 1000.02, 1e-10) &&
            near(state.pitchRateRadiansPerSecond, 0.1, 1e-10) &&
            near(
                state.pitchRadians,
                radians(1.75) + 0.01,
                1e-10
            ),
            "semi-implicit Euler integration"
        )) {
            return 1;
        }
    }

    // ========================================================
    // TEST 2: ELEVATOR AUTHORITY
    // ========================================================

    // A positive elevator input should create more
    // nose-up pitching moment than neutral elevator.
    {
        AircraftState state;

        Controls neutral;
        Controls pitchUp;

        pitchUp.elevator = 0.12;

        const Loads a =
            calculateLoads(state, aircraft, neutral);

        const Loads b =
            calculateLoads(state, aircraft, pitchUp);

        if (!check(
            b.pitchMomentNoseUpNm > a.pitchMomentNoseUpNm,
            "nose-up elevator authority"
        )) {
            return 1;
        }
    }

    // ========================================================
    // TEST 3: ATMOSPHERIC DENSITY
    // ========================================================

    // Air density should be approximately 1.225 kg/m^3
    // at sea level and decrease with altitude.
    {
        AircraftState state;

        state.positionMeters.z = 0.0;
        const Loads sea =
            calculateLoads(state, aircraft, Controls{});

        state.positionMeters.z = 1000.0;
        const Loads middle =
            calculateLoads(state, aircraft, Controls{});

        state.positionMeters.z = 5000.0;
        const Loads high =
            calculateLoads(state, aircraft, Controls{});

        if (!check(
            near(
                sea.airDensityKgPerCubicMeter,
                1.225,
                0.002
            ) &&
            sea.airDensityKgPerCubicMeter >
                middle.airDensityKgPerCubicMeter &&
            middle.airDensityKgPerCubicMeter >
                high.airDensityKgPerCubicMeter,
            "standard-atmosphere density"
        )) {
            return 1;
        }
    }

    // ========================================================
    // TEST 4: LIFT AND AIRSPEED
    // ========================================================

    // At the same altitude and angle of attack,
    // doubling airspeed should multiply lift by four.
    {
        AircraftState state;

        state.positionMeters.z = 0.0;
        state.pitchRadians = radians(5.0);

        state.velocityMetersPerSecond.x = 30.0;

        const Loads slow =
            calculateLoads(state, aircraft, Controls{});

        state.velocityMetersPerSecond.x = 60.0;

        const Loads fast =
            calculateLoads(state, aircraft, Controls{});

        if (!check(
            near(
                fast.liftNewtons,
                4.0 * slow.liftNewtons,
                1e-7
            ),
            "lift proportional to airspeed squared"
        )) {
            return 1;
        }
    }

    // ========================================================
    // TEST 5: STALL BEHAVIOR
    // ========================================================

    // Lift coefficient should increase before stall
    // and decrease after the critical angle.
    //
    // The model should also report increased post-stall
    // drag and activate its stall-state flag.
    {
        AircraftState state;

        state.velocityMetersPerSecond.x = 45.0;
        state.velocityMetersPerSecond.z = 0.0;

        state.pitchRadians = radians(5.0);
        const Loads five =
            calculateLoads(state, aircraft, Controls{});

        state.pitchRadians = radians(15.0);
        const Loads fifteen =
            calculateLoads(state, aircraft, Controls{});

        state.pitchRadians = radians(25.0);
        const Loads twentyFive =
            calculateLoads(state, aircraft, Controls{});

        if (!check(
            five.liftCoefficient <
                fifteen.liftCoefficient &&
            fifteen.liftCoefficient >
                twentyFive.liftCoefficient &&
            twentyFive.dragCoefficient >
                fifteen.dragCoefficient &&
            !five.isStalled &&
            fifteen.isStalled &&
            twentyFive.isStalled,
            "lift and drag across stall"
        )) {
            return 1;
        }
    }

    // ========================================================
    // TEST 6: INDUCED DRAG
    // ========================================================

    // With equal wing area, a wider wing has a larger
    // aspect ratio and produces less induced drag
    // in our simplified model.
    {
        AircraftState state;
        state.pitchRadians = radians(5.0);

        AircraftParameters widerWing = aircraft;
        widerWing.wingSpanMeters = 15.0;

        const Loads regular =
            calculateLoads(state, aircraft, Controls{});

        const Loads wider =
            calculateLoads(state, widerWing, Controls{});

        if (!check(
            wider.dragCoefficient <
                regular.dragCoefficient,
            "induced drag decreases with aspect ratio"
        )) {
            return 1;
        }
    }

    // ========================================================
    // TEST 7: PITCH DAMPING
    // ========================================================

    // Positive pitch rate should create a damping moment
    // that opposes nose-up rotation.
    {
        AircraftState state;

        const Loads still =
            calculateLoads(state, aircraft, Controls{});

        state.pitchRateRadiansPerSecond = 0.30;

        const Loads rotating =
            calculateLoads(state, aircraft, Controls{});

        if (!check(
            rotating.pitchMomentNoseUpNm <
                still.pitchMomentNoseUpNm,
            "pitch-rate damping"
        )) {
            return 1;
        }
    }

    // ========================================================
    // TEST 8: LIFT AT DIFFERENT ALTITUDES
    // ========================================================

    // At identical true airspeed and angle of attack,
    // lift should decrease as atmospheric density falls.
    {
        AircraftState state;

        state.pitchRadians = radians(5.0);

        state.positionMeters.z = 0.0;

        const Loads low =
            calculateLoads(state, aircraft, Controls{});

        state.positionMeters.z = 5000.0;

        const Loads high =
            calculateLoads(state, aircraft, Controls{});

        if (!check(
            low.liftNewtons > high.liftNewtons,
            "lift decreases with air density"
        )) {
            return 1;
        }
    }

    // ========================================================
    // TEST 9: FULL SIMULATION REGRESSION
    // ========================================================

    // Reproduce a known 12-second flight scenario.
    //
    // This catches unexpected numerical changes when
    // we modify the simulation in later versions.
    //
    // These reference values belong to Version 5.
    // They are NOT experimental aircraft measurements.
    {
        AircraftState state;
        Controls controls;

        for (int step = 0; step < 12 * 120; ++step) {

            const double t = step * kTimeStep;

            // Same elevator maneuver as our console program.
            controls.elevator =
                (t >= 2.0 && t < 2.5) ? 0.12 : 0.0;

            const Loads loads =
                calculateLoads(state, aircraft, controls);

            stepPhysics(
                state,
                aircraft,
                loads,
                kTimeStep
            );
        }

        if (!check(
            near(
                state.positionMeters.z,
                1010.53,
                0.03
            ) &&
            near(
                degrees(state.pitchRadians),
                0.51,
                0.03
            ),
            "12-second Version 5 regression"
        )) {
            return 1;
        }
    }

    std::cout
        << "PASS: All nine flight physics tests passed.\n";

    return 0;
}
