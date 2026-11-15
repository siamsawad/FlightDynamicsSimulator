
#include "FlightSim/Dynamics6DOF.hpp"
#include "FlightSim/SimulationConfig.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace flight::sixdof {

// ============================================================
// VECTOR MATHEMATICS
// ============================================================

Vec3 operator+(Vec3 a, Vec3 b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 operator-(Vec3 a, Vec3 b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 operator*(Vec3 a, double s) {
    return {a.x * s, a.y * s, a.z * s};
}

Vec3 operator/(Vec3 a, double s) {
    return {a.x / s, a.y / s, a.z / s};
}

double dot(Vec3 a, Vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 cross(Vec3 a, Vec3 b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

double magnitude(Vec3 a) {
    return std::sqrt(dot(a, a));
}

// ============================================================
// QUATERNION MULTIPLICATION
// ============================================================

Quaternion operator*(Quaternion a, Quaternion b) {

    return {
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
}

// Keep orientation quaternion at unit length.
Quaternion Quaternion::normalized() const {

    const double length = std::sqrt(
        w*w + x*x + y*y + z*z
    );

    if (!std::isfinite(length) || length < 1e-14) {
        throw std::invalid_argument(
            "Invalid orientation quaternion"
        );
    }

    return {
        w / length,
        x / length,
        y / length,
        z / length
    };
}

// ============================================================
// EULER ANGLES TO QUATERNION
// ============================================================
//
// Rotation convention: yaw, pitch, roll (ZYX).
//
// Positive pitch rotates the aircraft nose upward
// in world NED coordinates.
Quaternion Quaternion::fromEuler(
    double roll,
    double pitch,
    double yaw
) {

    const double cr = std::cos(roll / 2.0);
    const double sr = std::sin(roll / 2.0);

    const double cp = std::cos(pitch / 2.0);
    const double sp = std::sin(pitch / 2.0);

    const double cy = std::cos(yaw / 2.0);
    const double sy = std::sin(yaw / 2.0);

    return {
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy
    };
}

// ============================================================
// ROTATE BODY VECTOR INTO WORLD
// ============================================================

Vec3 rotateBodyToWorld(
    Quaternion rawQ,
    Vec3 v
) {

    const Quaternion q = rawQ.normalized();

    // Efficient equivalent of:
    //
    // q * (0, v) * conjugate(q)
    const Vec3 qvec{q.x, q.y, q.z};

    const Vec3 t = cross(qvec, v) * 2.0;

    return v + t * q.w + cross(qvec, t);
}

// ============================================================
// ROTATE WORLD VECTOR INTO BODY
// ============================================================

Vec3 rotateWorldToBody(
    Quaternion rawQ,
    Vec3 v
) {

    const Quaternion q = rawQ.normalized();

    return rotateBodyToWorld(
        {q.w, -q.x, -q.y, -q.z},
        v
    );
}

// ============================================================
// QUATERNION TO EULER ANGLES
// ============================================================

EulerAngles toEuler(Quaternion rawQ) {

    const Quaternion q = rawQ.normalized();

    EulerAngles a;

    a.rollRadians = std::atan2(
        2.0 * (q.w*q.x + q.y*q.z),
        1.0 - 2.0 * (q.x*q.x + q.y*q.y)
    );

    a.pitchRadians = std::asin(
        std::clamp(
            2.0 * (q.w*q.y - q.z*q.x),
            -1.0,
            1.0
        )
    );

    a.yawRadians = std::atan2(
        2.0 * (q.w*q.z + q.x*q.y),
        1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    );

    return a;
}

namespace {

// ============================================================
// STANDARD ATMOSPHERIC DENSITY
// ============================================================

double airDensity(double altitudeMeters) {

    constexpr double t0 = 288.15;
    constexpr double p0 = 101325.0;
    constexpr double lapse = 0.0065;
    constexpr double R = 287.05;

    const double h = std::clamp(
        altitudeMeters,
        0.0,
        11000.0
    );

    const double t = t0 - lapse * h;

    const double pressure = p0 * std::pow(
        t / t0,
        kGravity / (R * lapse)
    );

    return pressure / (R * t);
}

// ============================================================
// SMOOTH STALL TRANSITION
// ============================================================

double smoothStep(
    double a,
    double b,
    double x
) {

    const double t = std::clamp(
        (x - a) / (b - a),
        0.0,
        1.0
    );

    return t * t * (3.0 - 2.0 * t);
}

// Preserve the Version 5 lift curve.
double liftCoefficient(
    double alpha,
    const AircraftParameters& a
) {

    const double absAlpha = std::abs(alpha);

    const double stall = a.stallAngleRadians;

    const double linear =
        a.liftCoefficientAtZeroAoA
        + a.liftCurveSlopePerRadian * alpha;

    const double sign =
        alpha >= 0.0 ? 1.0 : -1.0;

    const double atStall =
        a.liftCoefficientAtZeroAoA
        + a.liftCurveSlopePerRadian * sign * stall;

    const double postStall =
        atStall * std::exp(
            -3.0 * std::max(
                0.0,
                absAlpha - stall
            )
        );

    const double blend = smoothStep(
        stall - radians(2.0),
        stall + radians(2.0),
        absAlpha
    );

    return (1.0 - blend) * linear
         + blend * postStall;
}

// ============================================================
// VALIDATE THE PHYSICAL MODEL
// ============================================================

void validateModel(
    const Model& m,
    const Wind& wind
) {

    const auto& a = m.aircraft;

    if (!(
        std::isfinite(a.massKg)
        && a.massKg > 0.0

        && std::isfinite(a.wingAreaM2)
        && a.wingAreaM2 > 0.0

        && std::isfinite(a.wingSpanMeters)
        && a.wingSpanMeters > 0.0

        && std::isfinite(a.meanChordMeters)
        && a.meanChordMeters > 0.0

        && std::isfinite(a.pitchInertiaKgM2)
        && a.pitchInertiaKgM2 > 0.0

        && std::isfinite(m.rollInertiaKgM2)
        && m.rollInertiaKgM2 > 0.0

        && std::isfinite(m.yawInertiaKgM2)
        && m.yawInertiaKgM2 > 0.0

        && std::isfinite(a.oswaldEfficiency)
        && a.oswaldEfficiency > 0.0

        && std::isfinite(wind.densityMultiplier)
        && wind.densityMultiplier > 0.0
    )) {

        throw std::invalid_argument(
            "Invalid 6-DOF mass, inertia, wing or atmosphere"
        );
    }
}

} // anonymous namespace

// ============================================================
// CALCULATE 3D FORCES AND MOMENTS
// ============================================================

ForcesMoments calculateForcesMoments(
    const State& state,
    const Model& model,
    const Inputs& inputs,
    const Wind& wind
) {

    validateModel(model, wind);

    const auto& a = model.aircraft;

    ForcesMoments out;

    // --------------------------------------------------------
    // 1. AIR-RELATIVE VELOCITY
    // --------------------------------------------------------

    // Subtract wind velocity from ground velocity.
    const Vec3 relativeWorld =
        state.velocityWorldMetersPerSecond
        - wind.velocityWorldMetersPerSecond;

    // Aerodynamic forces are calculated in body axes.
    const Vec3 relativeBody = rotateWorldToBody(
        state.bodyToWorld,
        relativeWorld
    );

    const double speed = magnitude(relativeBody);

    // --------------------------------------------------------
    // 2. ANGLE OF ATTACK AND SIDESLIP
    // --------------------------------------------------------

    // In FRD coordinates, positive body Z points downward.
    // Positive AoA therefore uses positive body Z velocity.
    const double alpha =
        speed > 1e-6
            ? std::atan2(
                relativeBody.z,
                relativeBody.x
            )
            : 0.0;

    // Sideslip measures the lateral component of airflow.
    const double beta =
        speed > 1e-6
            ? std::asin(
                std::clamp(
                    relativeBody.y / speed,
                    -1.0,
                    1.0
                )
            )
            : 0.0;

    // --------------------------------------------------------
    // 3. ATMOSPHERIC CONDITIONS
    // --------------------------------------------------------

    // World NED: altitude equals NEGATIVE position Z.
    const double rho =
        airDensity(-state.positionWorldMeters.z)
        * wind.densityMultiplier;

    const double qbar =
        0.5 * rho * speed * speed;

    // --------------------------------------------------------
    // 4. AERODYNAMIC COEFFICIENTS
    // --------------------------------------------------------

    const double aspectRatio =
        a.wingSpanMeters * a.wingSpanMeters
        / a.wingAreaM2;

    const double cl = liftCoefficient(alpha, a);

    const double excess = std::max(
        0.0,
        std::abs(alpha) -
            (
                a.stallAngleRadians
                - radians(2.0)
            )
    );

    const double cd =
        a.zeroLiftDragCoefficient
        + cl * cl / (
            kPi * aspectRatio * a.oswaldEfficiency
        )
        + 3.0 * excess * excess;

    // --------------------------------------------------------
    // 5. LIFT, DRAG, AND SIDE FORCE
    // --------------------------------------------------------

    const double lift =
        qbar * a.wingAreaM2 * cl;

    const double drag =
        qbar * a.wingAreaM2 * cd;

    const double side =
        qbar
        * a.wingAreaM2
        * model.sideForceBeta
        * beta;

    const double thrust =
        a.maxThrustNewtons
        * std::clamp(
            inputs.throttle,
            0.0,
            1.0
        );

    // Drag opposes the 3D air-relative velocity.
    const Vec3 dragForce =
        speed > 1e-6
            ? relativeBody * (-drag / speed)
            : Vec3{};

    // Lift acts upward in the aircraft's X/Z plane,
    // perpendicular to the forward/vertical airflow.
    const Vec3 liftForce{
        lift * std::sin(alpha),
        0.0,
        -lift * std::cos(alpha)
    };

    // Total body-frame aerodynamic and thrust forces.
    out.aerodynamicAndThrustForceBodyNewtons =
        dragForce
        + liftForce
        + Vec3{thrust, side, 0.0};

    // --------------------------------------------------------
    // 6. NONDIMENSIONAL ANGULAR RATES
    // --------------------------------------------------------

    const auto& w =
        state.angularVelocityBodyRadPerSecond;

    const double denom =
        2.0 * std::max(speed, 1.0);

    const double pHat =
        w.x * a.wingSpanMeters / denom;

    const double qHat =
        w.y * a.meanChordMeters / denom;

    const double rHat =
        w.z * a.wingSpanMeters / denom;

    // --------------------------------------------------------
    // 7. ROLL MOMENT
    // --------------------------------------------------------

    const double clRoll =
        model.rollMomentBeta * beta
        + model.rollMomentAileron
          * std::clamp(
              inputs.aileron,
              -1.0,
              1.0
          )
        + model.rollDamping * pHat;

    // --------------------------------------------------------
    // 8. PITCH MOMENT
    // --------------------------------------------------------

    // Preserve our earlier longitudinal coefficient model.
    //
    // The 0.12 factor maps normalized elevator input
    // to the older actuator's +/-0.12 range.
    const double cmPitch =
        a.pitchMomentAtZeroAoA
        + a.pitchStabilityPerRadian * alpha
        + a.elevatorMomentEffectiveness
          * 0.12
          * std::clamp(
              inputs.elevator,
              -1.0,
              1.0
          )
        + a.pitchDampingCoefficient * qHat;

    // --------------------------------------------------------
    // 9. YAW MOMENT
    // --------------------------------------------------------

    const double cnYaw =
        model.yawMomentBeta * beta
        + model.yawMomentRudder
          * std::clamp(
              inputs.rudder,
              -1.0,
              1.0
          )
        + model.yawDamping * rHat;

    // Convert moment coefficients into N*m.
    out.momentBodyNm = {

        qbar * a.wingAreaM2
             * a.wingSpanMeters * clRoll,

        qbar * a.wingAreaM2
             * a.meanChordMeters * cmPitch,

        qbar * a.wingAreaM2
             * a.wingSpanMeters * cnYaw
    };

    // --------------------------------------------------------
    // 10. ROTATE BODY FORCES INTO WORLD
    // --------------------------------------------------------

    // Body aerodynamic/thrust forces must be rotated.
    // Gravity already points along +Z in world NED.
    out.netForceWorldNewtons =
        rotateBodyToWorld(
            state.bodyToWorld,
            out.aerodynamicAndThrustForceBodyNewtons
        )
        + Vec3{
            0.0,
            0.0,
            a.massKg * kGravity
        };

    // --------------------------------------------------------
    // 11. STORE FLIGHT DATA
    // --------------------------------------------------------

    out.air = {
        speed,
        alpha,
        beta,
        rho,
        qbar,
        lift,
        drag,
        side,
        std::abs(alpha) >= a.stallAngleRadians
    };

    return out;
}

// ============================================================
// INTEGRATE FULL RIGID-BODY DYNAMICS
// ============================================================

void step(
    State& state,
    const Model& model,
    const Inputs& inputs,
    const Wind& wind,
    double dtSeconds
) {

    // Our existing physics frequency is 120 Hz.
    // Reject excessively large integration steps.
    if (
        !std::isfinite(dtSeconds)
        || dtSeconds <= 0.0
        || dtSeconds > 0.05
    ) {

        throw std::invalid_argument(
            "6-DOF timestep must be in (0, 0.05] s"
        );
    }

    // --------------------------------------------------------
    // 1. CALCULATE FORCES AND MOMENTS
    // --------------------------------------------------------

    const ForcesMoments f = calculateForcesMoments(
        state,
        model,
        inputs,
        wind
    );

    // F = m*a -> a = F/m.
    const Vec3 acceleration =
        f.netForceWorldNewtons
        / model.aircraft.massKg;

    // --------------------------------------------------------
    // 2. CALCULATE ROTATIONAL ACCELERATION
    // --------------------------------------------------------

    // Euler's rigid-body rotational equation:
    //
    // I * omegaDot = M - omega x (I * omega)
    //
    // Our first 6-DOF model uses a diagonal inertia tensor.
    const Vec3 omega =
        state.angularVelocityBodyRadPerSecond;

    const Vec3 inertia{
        model.rollInertiaKgM2,
        model.aircraft.pitchInertiaKgM2,
        model.yawInertiaKgM2
    };

    // Angular momentum: H = I * omega.
    const Vec3 angularMomentum{
        inertia.x * omega.x,
        inertia.y * omega.y,
        inertia.z * omega.z
    };

    // Subtract gyroscopic coupling.
    const Vec3 rhs =
        f.momentBodyNm
        - cross(omega, angularMomentum);

    const Vec3 omegaDot{
        rhs.x / inertia.x,
        rhs.y / inertia.y,
        rhs.z / inertia.z
    };

    // --------------------------------------------------------
    // 3. UPDATE TRANSLATIONAL STATE
    // --------------------------------------------------------

    // Semi-implicit Euler integration.
    state.velocityWorldMetersPerSecond =
        state.velocityWorldMetersPerSecond
        + acceleration * dtSeconds;

    state.positionWorldMeters =
        state.positionWorldMeters
        + state.velocityWorldMetersPerSecond
          * dtSeconds;

    // --------------------------------------------------------
    // 4. UPDATE ANGULAR VELOCITY
    // --------------------------------------------------------

    state.angularVelocityBodyRadPerSecond =
        state.angularVelocityBodyRadPerSecond
        + omegaDot * dtSeconds;

    // --------------------------------------------------------
    // 5. UPDATE ORIENTATION USING QUATERNIONS
    // --------------------------------------------------------

    const Vec3 rate =
        state.angularVelocityBodyRadPerSecond;

    const double angularSpeed =
        magnitude(rate);

    if (angularSpeed > 1e-12) {

        // Quaternion exponential rotation increment.
        const double halfAngle =
            0.5 * angularSpeed * dtSeconds;

        const double s =
            std::sin(halfAngle) / angularSpeed;

        const Quaternion dq{
            std::cos(halfAngle),
            rate.x * s,
            rate.y * s,
            rate.z * s
        };

        // Body angular rates multiply orientation
        // on the right.
        state.bodyToWorld =
            (state.bodyToWorld * dq).normalized();

    } else {

        state.bodyToWorld =
            state.bodyToWorld.normalized();
    }
}

// ============================================================
// EXISTING ENVIRONMENT -> 3D WIND
// ============================================================

Wind makeWind(const Environment& environment) {

    // A positive headwind flows toward -world X.
    return {
        {
            -environment.headwindKnots
             * kKnotsToMetersPerSecond,
            0.0,
            0.0
        },
        environment.airDensityMultiplier
    };
}

// ============================================================
// EXISTING SETTINGS -> INITIAL 6-DOF STATE
// ============================================================

State makeInitialState(
    const SimulationConfig& config
) {

    const auto checked =
        validateSimulationConfig(config);

    if (!checked.valid) {
        throw std::invalid_argument(
            checked.message
        );
    }

    State state;

    // NED uses negative Z for altitude above sea level.
    state.positionWorldMeters = {
        0.0,
        0.0,
        -config.initialAltitudeFeet
         * kFeetToMeters
    };

    // Preserve Version 8 initial true airspeed semantics.
    state.velocityWorldMetersPerSecond = {
        (
            config.initialAirspeedKnots
            - config.environment.headwindKnots
        ) * kKnotsToMetersPerSecond,
        0.0,
        0.0
    };

    // Retain our original starting pitch of 1.75 degrees.
    state.bodyToWorld = Quaternion::fromEuler(
        0.0,
        radians(1.75),
        0.0
    );

    return state;
}

} // namespace flight::sixdof
