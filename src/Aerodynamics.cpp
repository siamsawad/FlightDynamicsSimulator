#include "FlightSim/Aerodynamics.hpp"

#include <algorithm>
#include <cmath>

namespace flight {

namespace {

// ============================================================
// STANDARD ATMOSPHERIC DENSITY
// ============================================================

// Calculate density using our existing standard troposphere
// model, valid between sea level and 11,000 meters.
double airDensityAtAltitude(double altitudeMeters) {

    constexpr double seaLevelTemperature = 288.15;
    constexpr double seaLevelPressure = 101325.0;
    constexpr double lapseRate = 0.0065;
    constexpr double gasConstant = 287.05;

    const double altitude = std::clamp(
        altitudeMeters,
        0.0,
        11000.0
    );

    const double temperature =
        seaLevelTemperature - lapseRate * altitude;

    const double pressure =
        seaLevelPressure * std::pow(
            temperature / seaLevelTemperature,
            kGravity / (gasConstant * lapseRate)
        );

    return pressure / (gasConstant * temperature);
}

// ============================================================
// SMOOTH TRANSITION
// ============================================================

// Smoothly blend values between two boundaries.
//
// The curve is continuous and has zero slope at both
// boundaries, avoiding an abrupt change in the blend factor.
double smoothStep(
    double edge0,
    double edge1,
    double value
) {

    const double t = std::clamp(
        (value - edge0) / (edge1 - edge0),
        0.0,
        1.0
    );

    return t * t * (3.0 - 2.0 * t);
}

// ============================================================
// LIFT COEFFICIENT MODEL
// ============================================================

double liftCoefficient(
    double alpha,
    const AircraftParameters& aircraft
) {

    const double magnitude = std::abs(alpha);

    const double critical =
        aircraft.stallAngleRadians;

    // Linear lift curve before stall.
    const double linearCL =
        aircraft.liftCoefficientAtZeroAoA
        + aircraft.liftCurveSlopePerRadian * alpha;

    // Select the positive or negative stall boundary.
    const double sign = alpha >= 0.0 ? 1.0 : -1.0;

    const double clAtStall =
        aircraft.liftCoefficientAtZeroAoA
        + aircraft.liftCurveSlopePerRadian
          * sign * critical;

    // Simplified post-stall lift reduction.
    const double postStallCL =
        clAtStall * std::exp(
            -3.0 * std::max(
                0.0,
                magnitude - critical
            )
        );

    // Blend the two curves near the stall angle.
    const double blend = smoothStep(
        critical - radians(2.0),
        critical + radians(2.0),
        magnitude
    );

    return
        (1.0 - blend) * linearCL
        + blend * postStallCL;
}

} // anonymous namespace

// ============================================================
// MAIN AERODYNAMIC FUNCTION
// ============================================================

Loads calculateLoads(
    const AircraftState& state,
    const AircraftParameters& aircraft,
    const Controls& controls,
    const Environment& environment
) {

    // --------------------------------------------------------
    // 1. AIR-RELATIVE VELOCITY
    // --------------------------------------------------------

    // The physics engine stores ground-relative velocity.
    const double groundVx =
        state.velocityMetersPerSecond.x;

    const double groundVz =
        state.velocityMetersPerSecond.z;

    // Convert the user's headwind setting into m/s.
    //
    // Positive headwind points toward -X.
    // Air-relative velocity = ground velocity - wind velocity.
    //
    // Since wind velocity is negative for a headwind,
    // subtracting it increases forward air-relative speed.
    const double headwindMetersPerSecond =
        environment.headwindKnots *
        kKnotsToMetersPerSecond;

    const double airVx =
        groundVx + headwindMetersPerSecond;

    // No vertical wind is modeled in this version.
    const double airVz = groundVz;

    // True airspeed = magnitude of air-relative velocity.
    const double airspeed =
        std::hypot(airVx, airVz);

    // Air-relative flight-path angle.
    const double gamma =
        (airspeed > 0.001)
            ? std::atan2(airVz, airVx)
            : state.pitchRadians;

    // Angle of attack.
    const double alpha =
        state.pitchRadians - gamma;

    // --------------------------------------------------------
    // 2. ATMOSPHERIC DENSITY
    // --------------------------------------------------------

    const double standardDensity =
        airDensityAtAltitude(
            state.positionMeters.z
        );

    // Apply user-configured density multiplier.
    const double rho =
        standardDensity *
        environment.airDensityMultiplier;

    // Dynamic pressure.
    const double dynamicPressure =
        0.5 * rho * airspeed * airspeed;

    // --------------------------------------------------------
    // 3. LIFT COEFFICIENT
    // --------------------------------------------------------

    const double cl =
        liftCoefficient(alpha, aircraft);

    // --------------------------------------------------------
    // 4. INDUCED DRAG
    // --------------------------------------------------------

    const double aspectRatio =
        aircraft.wingSpanMeters *
        aircraft.wingSpanMeters /
        aircraft.wingAreaM2;

    const double inducedDragFactor =
        1.0 /
        (
            kPi *
            aspectRatio *
            aircraft.oswaldEfficiency
        );

    // Additional drag beyond the stall-transition region.
    const double excessAoA = std::max(
        0.0,
        std::abs(alpha) -
            (
                aircraft.stallAngleRadians
                - radians(2.0)
            )
    );

    const double extraStallDrag =
        3.0 * excessAoA * excessAoA;

    // Combined drag coefficient.
    const double cd =
        aircraft.zeroLiftDragCoefficient
        + inducedDragFactor * cl * cl
        + extraStallDrag;

    // --------------------------------------------------------
    // 5. LIFT AND DRAG FORCES
    // --------------------------------------------------------

    const double lift =
        dynamicPressure *
        aircraft.wingAreaM2 *
        cl;

    const double drag =
        dynamicPressure *
        aircraft.wingAreaM2 *
        cd;

    // --------------------------------------------------------
    // 6. ENGINE THRUST
    // --------------------------------------------------------

    // Preserve our existing simplified thrust model.
    const double thrust =
        aircraft.maxThrustNewtons *
        std::clamp(
            controls.throttle,
            0.0,
            1.0
        );

    // --------------------------------------------------------
    // 7. NET FORCES
    // --------------------------------------------------------

    Loads loads;

    // X direction: forward.
    loads.netForceNewtons.x =
        thrust * std::cos(state.pitchRadians)
        - drag * std::cos(gamma)
        - lift * std::sin(gamma);

    // Z direction: upward.
    //
    // Includes gravity, so the physics engine should NOT
    // subtract gravitational force a second time.
    loads.netForceNewtons.z =
        thrust * std::sin(state.pitchRadians)
        - drag * std::sin(gamma)
        + lift * std::cos(gamma)
        - aircraft.massKg * kGravity;

    // --------------------------------------------------------
    // 8. PITCHING MOMENT
    // --------------------------------------------------------

    // Nondimensional pitch-rate term.
    const double normalizedPitchRate =
        (airspeed > 0.001)
            ? state.pitchRateRadiansPerSecond *
                  aircraft.meanChordMeters /
                  (2.0 * airspeed)
            : 0.0;

    // Existing longitudinal stability model.
    const double cm =
        aircraft.pitchMomentAtZeroAoA
        + aircraft.pitchStabilityPerRadian * alpha
        + aircraft.elevatorMomentEffectiveness *
          std::clamp(
              controls.elevator,
              -1.0,
              1.0
          )
        + aircraft.pitchDampingCoefficient *
          normalizedPitchRate;

    loads.pitchMomentNoseUpNm =
        dynamicPressure *
        aircraft.wingAreaM2 *
        aircraft.meanChordMeters *
        cm;

    // --------------------------------------------------------
    // 9. RECORD AERODYNAMIC MEASUREMENTS
    // --------------------------------------------------------

    loads.airspeedMetersPerSecond = airspeed;
    loads.angleOfAttackRadians = alpha;

    loads.airDensityKgPerCubicMeter = rho;
    loads.dynamicPressurePascals = dynamicPressure;

    loads.liftNewtons = lift;
    loads.dragNewtons = drag;
    loads.thrustNewtons = thrust;

    loads.liftCoefficient = cl;
    loads.dragCoefficient = cd;

    loads.isStalled =
        std::abs(alpha) >= aircraft.stallAngleRadians;

    return loads;
}

} // namespace flight