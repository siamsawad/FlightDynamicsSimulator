
#include "FlightSim/Autopilot6DOF.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace flight::sixdof {

namespace {

constexpr double kTwoPi = 2.0 * kPi;

// Return the shortest angular difference.
// Handles crossing between 359 and 0 degrees.
double wrapAngle(double angle) {

    return std::remainder(
        angle,
        kTwoPi
    );
}

bool finite(double value) {

    return std::isfinite(value);
}

} // anonymous namespace

// ============================================================
// 1. ENGAGE AUTOPILOT
// ============================================================

void Autopilot6DOF::engage(
    const State& state
) {

    const EulerAngles attitude =
        toEuler(state.bodyToWorld);

    const double altitude =
        -state.positionWorldMeters.z;

    if (!finite(altitude)) {

        throw std::invalid_argument(
            "Invalid altitude on autopilot engage"
        );
    }

    // Reset previous controller state.
    status_ = Autopilot6DOFStatus{};

    status_.engaged = true;

    // Capture current altitude.
    status_.targetAltitudeMeters =
        std::clamp(
            altitude,
            0.0,
            11000.0
        );

    // Capture current heading.
    status_.targetHeadingRadians =
        wrapAngle(attitude.yawRadians);

    altitudeIntegralMeterSeconds_ = 0.0;
}

// ============================================================
// 2. DISENGAGE AUTOPILOT
// ============================================================

void Autopilot6DOF::disengage() {

    status_.engaged = false;

    status_.lowAirspeed = false;

    status_.highAngleOfAttack = false;

    status_.requestedAxes = PilotAxes{};

    altitudeIntegralMeterSeconds_ = 0.0;
}

// ============================================================
// 3. ALTITUDE TARGET
// ============================================================

void Autopilot6DOF::setTargetAltitudeMeters(
    double target
) {

    if (!finite(target)) {

        throw std::invalid_argument(
            "Target altitude must be finite"
        );
    }

    status_.targetAltitudeMeters = std::clamp(
        target,
        0.0,
        11000.0
    );
}

void Autopilot6DOF::adjustTargetAltitudeMeters(
    double delta
) {

    if (!finite(delta)) {

        throw std::invalid_argument(
            "Invalid altitude adjustment"
        );
    }

    setTargetAltitudeMeters(
        status_.targetAltitudeMeters + delta
    );
}

// ============================================================
// 4. HEADING TARGET
// ============================================================

void Autopilot6DOF::setTargetHeadingRadians(
    double heading
) {

    if (!finite(heading)) {

        throw std::invalid_argument(
            "Target heading must be finite"
        );
    }

    status_.targetHeadingRadians =
        wrapAngle(heading);
}

void Autopilot6DOF::adjustTargetHeadingRadians(
    double delta
) {

    if (!finite(delta)) {

        throw std::invalid_argument(
            "Invalid heading adjustment"
        );
    }

    setTargetHeadingRadians(
        status_.targetHeadingRadians + delta
    );
}

// ============================================================
// 5. UPDATE SIX-DEGREE-OF-FREEDOM AUTOPILOT
// ============================================================

PilotAxes Autopilot6DOF::update(
    const State& state,
    const AirData& air,
    const Model& model,
    double dtSeconds
) {

    // Disengaged autopilot produces neutral requests.
    if (!status_.engaged) {
        return {};
    }

    // --------------------------------------------------------
    // A. VALIDATE INPUTS
    // --------------------------------------------------------

    if (
        !finite(dtSeconds) ||
        dtSeconds <= 0.0 ||
        dtSeconds > 0.05
    ) {

        throw std::invalid_argument(
            "Autopilot timestep must be in (0, 0.05] s"
        );
    }

    if (
        !finite(air.trueAirspeedMetersPerSecond) ||
        !finite(air.angleOfAttackRadians) ||
        !finite(air.sideslipRadians) ||
        !finite(air.densityKgPerCubicMeter) ||
        air.densityKgPerCubicMeter <= 0.0
    ) {

        throw std::invalid_argument(
            "Invalid air data for autopilot"
        );
    }

    // --------------------------------------------------------
    // B. MEASURE CURRENT FLIGHT CONDITIONS
    // --------------------------------------------------------

    const EulerAngles attitude =
        toEuler(state.bodyToWorld);

    const Vec3 rates =
        state.angularVelocityBodyRadPerSecond;

    // World NED:
    // Negative Z is positive altitude.
    const double currentAltitude =
        -state.positionWorldMeters.z;

    // Positive climb rate means altitude increasing.
    const double climbRate =
        -state.velocityWorldMetersPerSecond.z;

    const double altitudeError =
        status_.targetAltitudeMeters -
        currentAltitude;

    const double headingError =
        wrapAngle(
            status_.targetHeadingRadians -
            attitude.yawRadians
        );

    status_.altitudeErrorMeters = altitudeError;

    status_.headingErrorRadians = headingError;

    // --------------------------------------------------------
    // C. ESTIMATE LOW-AIRSPEED / HIGH-AOA CONDITIONS
    // --------------------------------------------------------

    const auto& a = model.aircraft;

    if (
        !finite(a.massKg) ||
        a.massKg <= 0.0 ||
        !finite(a.wingAreaM2) ||
        a.wingAreaM2 <= 0.0
    ) {

        throw std::invalid_argument(
            "Invalid mass or wing area for autopilot"
        );
    }

    // Approximate pre-stall reference lift coefficient.
    const double clReference = std::max(
        0.2,
        a.liftCoefficientAtZeroAoA
        + a.liftCurveSlopePerRadian *
          (
              a.stallAngleRadians -
              radians(2.0)
          )
    );

    const double stallSpeedEstimate =
        std::sqrt(
            (
                2.0 *
                a.massKg *
                kGravity
            ) /
            (
                air.densityKgPerCubicMeter *
                a.wingAreaM2 *
                clReference
            )
        );

    status_.minimumRecommendedAirspeedMetersPerSecond =
        1.30 * stallSpeedEstimate;

    status_.lowAirspeed =
        air.trueAirspeedMetersPerSecond <
        status_.minimumRecommendedAirspeedMetersPerSecond;

    status_.highAngleOfAttack =
        std::abs(
            air.angleOfAttackRadians
        ) >= radians(13.0);

    // ========================================================
    // D. VERSION 13 ALTITUDE CONTROLLER
    // ========================================================

    // The controller now has two stages:
    //
    // Altitude error -> desired climb rate
    // Desired climb rate -> desired pitch
    //
    // This produces a more deliberate climb and allows
    // the aircraft to level gradually near its target.

    // --------------------------------------------------------
    // D1. LIMITED INTEGRAL CORRECTION
    // --------------------------------------------------------

    // Only accumulate error under normal conditions.
    //
    // The integral helps correct small persistent
    // altitude errors without growing indefinitely.
    if (
        !status_.lowAirspeed &&
        !status_.highAngleOfAttack &&
        std::abs(altitudeError) < 75.0
    ) {

        altitudeIntegralMeterSeconds_ =
            std::clamp(
                altitudeIntegralMeterSeconds_
                    + altitudeError * dtSeconds,
                -120.0,
                120.0
            );
    }

    // --------------------------------------------------------
    // D2. ALTITUDE ERROR TO DESIRED CLIMB RATE
    // --------------------------------------------------------

    // Maximum requested vertical speed:
    // 2.3 m/s = approximately 453 ft/min.
    constexpr double kAltitudeToClimbGain = 0.30;

    constexpr double kMaxClimbRateMetersPerSecond =
        2.3;

    const double desiredClimbRate = std::clamp(
        kAltitudeToClimbGain * altitudeError,
        -kMaxClimbRateMetersPerSecond,
        kMaxClimbRateMetersPerSecond
    );

    // --------------------------------------------------------
    // D3. VERTICAL SPEED ERROR TO DESIRED PITCH
    // --------------------------------------------------------

    const double climbRateError =
        desiredClimbRate - climbRate;

    constexpr double kClimbRateToPitchGain = 0.10;

    constexpr double kAltitudeIntegralGain =
        0.00012;

    double desiredPitch = std::clamp(
        radians(1.75)
        + kClimbRateToPitchGain * climbRateError
        + kAltitudeIntegralGain *
          altitudeIntegralMeterSeconds_,
        radians(-10.0),
        radians(12.0)
    );

    // ========================================================
    // E. PITCH INNER LOOP
    // ========================================================

    // Elevator controls the difference between desired
    // and actual pitch while pitch-rate feedback damps
    // rotational motion.
    double elevator = std::clamp(
        3.5 *
        (
            desiredPitch -
            attitude.pitchRadians
        )
        - 1.4 * rates.y,
        -1.0,
        1.0
    );

    // ========================================================
    // F. HEADING OUTER LOOP
    // ========================================================

    // Desired heading determines desired bank.
    double desiredBank = std::clamp(
        0.75 * headingError
        - 0.25 * rates.z,
        radians(-20.0),
        radians(20.0)
    );

    // ========================================================
    // G. ROLL INNER LOOP
    // ========================================================

    // Ailerons track bank demand with roll-rate damping.
    double aileron = std::clamp(
        2.8 *
        (
            desiredBank -
            attitude.rollRadians
        )
        - 0.85 * rates.x,
        -1.0,
        1.0
    );

    // ========================================================
    // H. RUDDER / SIDESLIP CONTROL
    // ========================================================

    // Rudder helps track heading and reduce sideslip.
    double rudder = std::clamp(
        0.35 * headingError
        - 1.2 * air.sideslipRadians
        - 0.45 * rates.z,
        -0.5,
        0.5
    );

    // ========================================================
    // I. SIMPLIFIED FLIGHT-ENVELOPE RESPONSE
    // ========================================================

    // These protections are educational approximations,
    // not guaranteed stall-prevention systems.

    if (status_.lowAirspeed) {

        // Limit nose-up request at low airspeed.
        desiredPitch = std::min(
            desiredPitch,
            radians(0.0)
        );

        elevator = std::min(
            elevator,
            -0.1
        );

        desiredBank = std::clamp(
            desiredBank,
            radians(-10.0),
            radians(10.0)
        );
    }

    if (status_.highAngleOfAttack) {

        // Attempt to reduce excessive AoA.
        elevator =
            air.angleOfAttackRadians > 0.0
                ? -0.8
                : 0.8;

        // Attempt to return the wings toward level.
        desiredBank = 0.0;

        aileron = std::clamp(
            -2.8 * attitude.rollRadians
            - 0.85 * rates.x,
            -1.0,
            1.0
        );
    }

    // ========================================================
    // J. RETURN THE CONTROL COMMANDS
    // ========================================================

    status_.desiredPitchRadians = desiredPitch;

    status_.desiredBankRadians = desiredBank;

    // Throttle remains under manual control.
    // W/S from the GUI still manage engine power.
    status_.requestedAxes = {
        elevator,
        aileron,
        rudder,
        0.0
    };

    return status_.requestedAxes;
}

} // namespace flight::sixdof
