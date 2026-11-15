
#include "FlightSim/Dynamics6DOF.hpp"
#include "FlightSim/SimulationConfig.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace flight;
using namespace flight::sixdof;

namespace {

bool near(
    double a,
    double b,
    double epsilon = 1e-8
) {
    return std::abs(a - b) <= epsilon;
}

bool check(
    bool value,
    const char* description
) {

    if (!value) {
        std::cerr
            << "FAIL: "
            << description
            << '\n';
    }

    return value;
}

bool finite(Vec3 v) {
    return
        std::isfinite(v.x)
        && std::isfinite(v.y)
        && std::isfinite(v.z);
}

} // anonymous namespace

int main() {

    Model model;
    Wind wind;
    Inputs inputs;
    State state;

    // ========================================================
    // TEST 1: IDENTITY QUATERNION
    // ========================================================

    if (!check(
        near(
            rotateBodyToWorld(
                {},
                {1, 2, 3}
            ).x,
            1
        ),
        "Identity rotation"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 2: POSITIVE YAW
    // ========================================================

    auto yaw = Quaternion::fromEuler(
        0,
        0,
        radians(90)
    );

    Vec3 forward = rotateBodyToWorld(
        yaw,
        {1, 0, 0}
    );

    if (!check(
        near(forward.x, 0)
        && near(forward.y, 1),
        "Positive yaw direction"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 3: POSITIVE PITCH
    // ========================================================

    auto pitch = Quaternion::fromEuler(
        0,
        radians(30),
        0
    );

    forward = rotateBodyToWorld(
        pitch,
        {1, 0, 0}
    );

    if (!check(
        near(forward.z, -0.5),
        "Positive pitch direction"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 4: INVERSE ROTATION
    // ========================================================

    auto orientation =
        Quaternion::fromEuler(0.4, -0.2, 0.8);

    const Vec3 original{3, -7, 2};

    const Vec3 recovered = rotateWorldToBody(
        orientation,
        rotateBodyToWorld(
            orientation,
            original
        )
    );

    if (!check(
        near(recovered.x, original.x)
        && near(recovered.y, original.y)
        && near(recovered.z, original.z),
        "Inverse rotation"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 5: GRAVITY
    // ========================================================

    State falling;

    falling.velocityWorldMetersPerSecond = {
        0, 0, 0
    };

    Inputs noThrust;
    noThrust.throttle = 0;

    step(
        falling,
        model,
        noThrust,
        wind,
        0.02
    );

    if (!check(
        near(
            falling.velocityWorldMetersPerSecond.z,
            0.1962
        )
        && near(
            falling.positionWorldMeters.z,
            -999.996076
        ),
        "Gravity in NED coordinates"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 6: THRUST ACCELERATION
    // ========================================================

    State thrustOnly;

    thrustOnly.velocityWorldMetersPerSecond = {
        0, 0, 0
    };

    inputs.throttle = 1;

    step(
        thrustOnly,
        model,
        inputs,
        wind,
        0.02
    );

    if (!check(
        near(
            thrustOnly.velocityWorldMetersPerSecond.x,
            0.07
        ),
        "Forward thrust"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 7: AILERON GENERATES ROLL MOMENT
    // ========================================================

    inputs = Inputs{};
    inputs.aileron = 0.25;

    const ForcesMoments rollLoads =
        calculateForcesMoments(
            state,
            model,
            inputs,
            wind
        );

    if (!check(
        rollLoads.momentBodyNm.x > 0,
        "Aileron creates roll"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 8: RUDDER GENERATES YAW MOMENT
    // ========================================================

    inputs = Inputs{};
    inputs.rudder = 0.25;

    const ForcesMoments yawLoads =
        calculateForcesMoments(
            state,
            model,
            inputs,
            wind
        );

    if (!check(
        yawLoads.momentBodyNm.z > 0,
        "Rudder creates yaw"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 9: ELEVATOR GENERATES PITCH MOMENT
    // ========================================================

    inputs = Inputs{};

    const double neutralPitchMoment =
        calculateForcesMoments(
            state,
            model,
            inputs,
            wind
        ).momentBodyNm.y;

    inputs.elevator = 0.5;

    if (!check(
        calculateForcesMoments(
            state,
            model,
            inputs,
            wind
        ).momentBodyNm.y > neutralPitchMoment,
        "Elevator creates pitch"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 10: SIDESLIP FORCE
    // ========================================================

    State slipping;

    slipping.velocityWorldMetersPerSecond = {
        50, 5, 0
    };

    inputs = Inputs{};

    const auto sideLoads =
        calculateForcesMoments(
            slipping,
            model,
            inputs,
            wind
        );

    if (!check(
        sideLoads.air.sideslipRadians > 0
        && sideLoads.air.sideForceNewtons < 0,
        "Sideslip force direction"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 11: GYROSCOPIC COUPLING
    // ========================================================

    State coupling;

    coupling.velocityWorldMetersPerSecond = {
        0, 0, 0
    };

    coupling.angularVelocityBodyRadPerSecond = {
        0.5, 0.7, 0
    };

    Model noAero = model;
    noAero.aircraft.maxThrustNewtons = 0;

    step(
        coupling,
        noAero,
        noThrust,
        wind,
        0.01
    );

    if (!check(
        coupling.angularVelocityBodyRadPerSecond.z < 0,
        "Rigid-body gyroscopic coupling"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 12: QUATERNION NORMALIZATION
    // ========================================================

    State maneuver;

    inputs = Inputs{};
    inputs.aileron = 0.20;
    inputs.rudder = 0.05;

    for (int i = 0; i < 240; ++i) {

        step(
            maneuver,
            model,
            inputs,
            wind,
            kTimeStep
        );
    }

    const auto& q = maneuver.bodyToWorld;

    const double norm = std::sqrt(
        q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z
    );

    if (!check(
        near(norm, 1.0, 1e-10),
        "Quaternion stays normalized"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 13: HEADWIND AND TRUE AIRSPEED
    // ========================================================

    State windy;

    windy.velocityWorldMetersPerSecond = {
        40, 0, 0
    };

    wind.velocityWorldMetersPerSecond = {
        -10, 0, 0
    };

    const auto withWind =
        calculateForcesMoments(
            windy,
            model,
            Inputs{},
            wind
        );

    if (!check(
        near(
            withWind.air.trueAirspeedMetersPerSecond,
            50
        ),
        "3D air-relative velocity"
    )) {
        return 1;
    }

    wind = Wind{};

    // ========================================================
    // TEST 14: CONFIGURATION COMPATIBILITY
    // ========================================================

    SimulationConfig config;

    config.initialAltitudeFeet = 5000;
    config.environment.headwindKnots = 20;

    const State configured =
        makeInitialState(config);

    const Wind configuredWind =
        makeWind(config.environment);

    const auto initialLoads =
        calculateForcesMoments(
            configured,
            model,
            Inputs{},
            configuredWind
        );

    if (!check(
        near(
            configured.positionWorldMeters.z,
            -1524
        )
        && near(
            initialLoads.air.trueAirspeedMetersPerSecond,
            50
        ),
        "Existing settings compatibility"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 15: LONGITUDINAL FORCE COMPATIBILITY
    // ========================================================

    // With zero wind, zero bank, and zero yaw, our new
    // 6-DOF forces should match the earlier X/Z model.

    State comparison;

    comparison.bodyToWorld =
        Quaternion::fromEuler(
            0,
            radians(1.75),
            0
        );

    const auto c =
        calculateForcesMoments(
            comparison,
            model,
            Inputs{},
            Wind{}
        );

    const double theta = radians(1.75);

    const double predictedFx =
        Inputs{}.throttle
        * model.aircraft.maxThrustNewtons
        * std::cos(theta)
        - c.air.dragNewtons;

    const double predictedFzDown =
        model.aircraft.massKg * kGravity
        - Inputs{}.throttle
          * model.aircraft.maxThrustNewtons
          * std::sin(theta)
        - c.air.liftNewtons;

    if (!check(
        near(
            c.netForceWorldNewtons.x,
            predictedFx,
            1e-7
        )
        && near(
            c.netForceWorldNewtons.z,
            predictedFzDown,
            1e-7
        ),
        "Longitudinal force compatibility"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 16: ALL SIX DEGREES OF FREEDOM ACTIVE
    // ========================================================

    State sixAxes;

    sixAxes.bodyToWorld =
        Quaternion::fromEuler(
            0,
            radians(1.75),
            0
        );

    Inputs combined;
    combined.aileron = 0.12;
    combined.rudder = 0.08;

    for (int i = 0; i < 120; ++i) {

        step(
            sixAxes,
            model,
            combined,
            Wind{},
            kTimeStep
        );
    }

    const auto angles =
        toEuler(sixAxes.bodyToWorld);

    if (!check(
        finite(sixAxes.positionWorldMeters)
        && finite(sixAxes.velocityWorldMetersPerSecond)
        && std::abs(
            sixAxes.positionWorldMeters.y
        ) > 1e-6
        && std::abs(angles.rollRadians) > 1e-6
        && std::abs(angles.pitchRadians) > 1e-6
        && std::abs(angles.yawRadians) > 1e-6,
        "All six degrees of freedom active"
    )) {
        return 1;
    }

    // ========================================================
    // TEST 17: INVALID INERTIA REJECTED
    // ========================================================

    bool caught = false;

    try {

        Model bad = model;
        bad.rollInertiaKgM2 = 0;

        calculateForcesMoments(
            State{},
            bad,
            Inputs{}
        );

    } catch (const std::invalid_argument&) {

        caught = true;
    }

    if (!check(
        caught,
        "Invalid inertia rejected"
    )) {
        return 1;
    }

    std::cout
        << "PASS: All 17 six-degree-of-freedom "
        << "checks passed.\n";

    return 0;
}
