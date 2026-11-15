
#include "FlightSim/FlightEnvelope.hpp"

#include <algorithm>
#include <cmath>

namespace flight {

// ============================================================
// RESET PROTECTION STATE
// ============================================================

void FlightEnvelopeProtection::reset() {

    lowAirspeedActive_ = false;
    stallRecoveryActive_ = false;
}

// ============================================================
// EVALUATE AIRCRAFT FLIGHT ENVELOPE
// ============================================================

EnvelopeStatus FlightEnvelopeProtection::update(
    const AircraftState& state,
    const AircraftParameters& aircraft,
    const Loads& loads,
    double requestedElevatorAxis,
    bool autopilotEngaged
) {

    EnvelopeStatus status;

    // Begin with the original autopilot request.
    status.appliedElevatorAxis = std::clamp(
        requestedElevatorAxis,
        -1.0,
        1.0
    );

    // --------------------------------------------------------
    // 1. ESTIMATE AN AIRSPEED BOUNDARY
    // --------------------------------------------------------

    // Use the pre-stall lift coefficient near 13 degrees.
    //
    // This estimate depends on aircraft mass,
    // wing area, and atmospheric density.
    const double representativeCL = std::max(
        0.2,
        aircraft.liftCoefficientAtZeroAoA
        + aircraft.liftCurveSlopePerRadian
          * (
              aircraft.stallAngleRadians
              - radians(2.0)
            )
    );

    // Avoid division by a zero density.
    const double rho = std::max(
        0.01,
        loads.airDensityKgPerCubicMeter
    );

    // Estimate the reference stall speed.
    const double referenceStallSpeed = std::sqrt(
        2.0 * aircraft.massKg * kGravity /
        (
            rho
            * aircraft.wingAreaM2
            * representativeCL
        )
    );

    // Add a 30% airspeed margin.
    const double speedThreshold =
        1.30 * referenceStallSpeed;

    status.minimumRecommendedAirspeedMetersPerSecond =
        speedThreshold;

    // --------------------------------------------------------
    // 2. MANUAL CONTROL BYPASSES PROTECTION
    // --------------------------------------------------------

    if (!autopilotEngaged) {

        reset();
        return status;
    }

    // --------------------------------------------------------
    // 3. LOW-AIRSPEED PROTECTION
    // --------------------------------------------------------

    // Engage below the recommended threshold.
    if (!lowAirspeedActive_ &&
        loads.airspeedMetersPerSecond < speedThreshold) {

        lowAirspeedActive_ = true;
    }

    // Disengage only after speed increases sufficiently.
    else if (
        lowAirspeedActive_
        && loads.airspeedMetersPerSecond >
            1.10 * speedThreshold
    ) {

        lowAirspeedActive_ = false;
    }

    // --------------------------------------------------------
    // 4. STALL RECOVERY MODE
    // --------------------------------------------------------

    const double absAlpha =
        std::abs(loads.angleOfAttackRadians);

    // Enter recovery when the AoA reaches the nominal
    // stall boundary from our aircraft model.
    if (
        !stallRecoveryActive_
        && absAlpha >= aircraft.stallAngleRadians
    ) {

        stallRecoveryActive_ = true;
    }

    // Leave recovery after the AoA has reduced by
    // at least four degrees below that boundary.
    else if (
        stallRecoveryActive_
        && absAlpha <
            aircraft.stallAngleRadians - radians(4.0)
    ) {

        stallRecoveryActive_ = false;
    }

    status.lowAirspeed = lowAirspeedActive_;
    status.stallRecovery = stallRecoveryActive_;

    // --------------------------------------------------------
    // 5. OVERRIDE THE AUTOPILOT WHEN NECESSARY
    // --------------------------------------------------------

    if (stallRecoveryActive_) {

        // Excessive positive AoA -> nose-down request.
        // Excessive negative AoA -> nose-up request.
        //
        // This attempts to reduce the magnitude of AoA.
        status.appliedElevatorAxis =
            loads.angleOfAttackRadians >= 0.0
                ? -1.0
                : +1.0;
    }

    else if (
        lowAirspeedActive_
        && state.positionMeters.z > 1.0
    ) {

        // While airborne and below the speed boundary,
        // prevent additional nose-up AP requests.
        //
        // This is a basic intervention, not a guarantee
        // of stall prevention or successful recovery.
        status.appliedElevatorAxis = std::min(
            status.appliedElevatorAxis,
            0.0
        );
    }

    return status;
}

} // namespace flight
