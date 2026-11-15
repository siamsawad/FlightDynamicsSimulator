
#include "FlightSim/GroundContact.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace flight {

// ============================================================
// CHECK AND RESOLVE GROUND CONTACT
// ============================================================

GroundContactReport GroundContact::resolve(
    AircraftState& state,
    const AircraftParameters& aircraft,
    const Loads& loads,
    double dtSeconds
) {

    // Reject invalid physics time steps.
    if (!std::isfinite(dtSeconds) || dtSeconds <= 0.0) {

        throw std::invalid_argument(
            "Ground contact requires a positive finite timestep"
        );
    }

    GroundContactReport report;

    // --------------------------------------------------------
    // 1. CHECK IF AIRCRAFT IS ABOVE THE RUNWAY
    // --------------------------------------------------------

    if (state.positionMeters.z > kRunwayAltitudeMeters) {

        // Nothing to correct while airborne.
        wasOnGround_ = false;

        return report;
    }

    // --------------------------------------------------------
    // 2. MEASURE THE TOUCHDOWN SPEED
    // --------------------------------------------------------

    // Store the vertical speed before stopping downward
    // motion. A negative vertical speed means descent.
    const double impactSpeed = std::max(
        0.0,
        -state.velocityMetersPerSecond.z
    );

    // --------------------------------------------------------
    // 3. ENFORCE THE RUNWAY BOUNDARY
    // --------------------------------------------------------

    // Do not allow the aircraft's reference point
    // to pass below our flat runway at sea level.
    state.positionMeters.z = kRunwayAltitudeMeters;

    // Stop any downward velocity.
    //
    // A positive vertical velocity is preserved
    // so the aircraft can leave the ground.
    state.velocityMetersPerSecond.z = std::max(
        0.0,
        state.velocityMetersPerSecond.z
    );

    report.onGround = true;

    // First contact after being airborne.
    report.touchedDown = !wasOnGround_;

    // Record impact speed for a newly detected touchdown.
    report.impactSpeedMetersPerSecond =
        report.touchedDown ? impactSpeed : 0.0;

    // Flag relatively high vertical touchdown speed.
    //
    // The 3 m/s boundary is a project-defined
    // classification, not a real aircraft limit.
    report.hardLanding =
        report.touchedDown && impactSpeed > 3.0;

    wasOnGround_ = true;

    // --------------------------------------------------------
    // 4. APPROXIMATE NORMAL FORCE
    // --------------------------------------------------------

    // loads.netForceNewtons.z already includes gravity.
    //
    // If the net force points downward, the ground
    // provides an opposing normal force.
    report.normalForceNewtons = std::max(
        0.0,
        -loads.netForceNewtons.z
    );

    // --------------------------------------------------------
    // 5. CALCULATE ROLLING RESISTANCE
    // --------------------------------------------------------

    report.rollingResistanceNewtons =
        kRollingResistanceCoefficient
        * report.normalForceNewtons;

    // Convert resistance force into speed reduction:
    //
    // deltaV = (F / mass) * dt
    const double maxSpeedReduction =
        report.rollingResistanceNewtons
        / aircraft.massKg
        * dtSeconds;

    const double vx =
        state.velocityMetersPerSecond.x;

    // Resistance opposes forward motion.
    if (vx > 0.0) {

        state.velocityMetersPerSecond.x = std::max(
            0.0,
            vx - maxSpeedReduction
        );
    }

    // Resistance also opposes backward motion.
    else if (vx < 0.0) {

        state.velocityMetersPerSecond.x = std::min(
            0.0,
            vx + maxSpeedReduction
        );
    }

    // Return the ground-contact measurements.
    return report;
}

} // namespace flight
