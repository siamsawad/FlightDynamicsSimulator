
#include "FlightSim/Autopilot.hpp"

#include <algorithm> // std::clamp
#include <cmath>      // std::isfinite
#include <stdexcept>  // std::invalid_argument

namespace flight {

// ============================================================
// ENGAGE AUTOPILOT
// ============================================================

void Autopilot::engage(const AircraftState& state) {

    // Capture the current altitude as the initial target.
    //
    // This prevents a sudden target change when the pilot
    // switches from manual control to altitude hold.
    setTargetAltitudeMeters(state.positionMeters.z);

    // Clear accumulated error from any previous engagement.
    accumulatedErrorMeterSeconds_ = 0.0;

    engaged_ = true;
}

// ============================================================
// DISENGAGE AUTOPILOT
// ============================================================

void Autopilot::disengage() {

    // Manual flight can now control the elevator.
    engaged_ = false;

    // Remove accumulated error so an old integral value
    // doesn't affect the next autopilot engagement.
    accumulatedErrorMeterSeconds_ = 0.0;
}

// ============================================================
// CHANGE TARGET ALTITUDE
// ============================================================

void Autopilot::setTargetAltitudeMeters(
    double altitudeMeters
) {

    // Reject invalid numerical inputs.
    if (!std::isfinite(altitudeMeters)) {
        throw std::invalid_argument(
            "Autopilot altitude target must be finite"
        );
    }

    // Our Version 5 atmospheric model covers 0 to 11 km.
    // Keep the commanded altitude in that model range.
    targetAltitudeMeters_ = std::clamp(
        altitudeMeters,
        0.0,
        11000.0
    );
}

void Autopilot::adjustTargetAltitudeMeters(
    double deltaMeters
) {

    if (!std::isfinite(deltaMeters)) {
        throw std::invalid_argument(
            "Autopilot target adjustment must be finite"
        );
    }

    setTargetAltitudeMeters(
        targetAltitudeMeters_ + deltaMeters
    );
}

// ============================================================
// CALCULATE ALTITUDE ERROR
// ============================================================

double Autopilot::altitudeErrorMeters(
    const AircraftState& state
) const {

    return targetAltitudeMeters_
        - state.positionMeters.z;
}

// ============================================================
// PID CONTROLLER
// ============================================================

double Autopilot::update(
    const AircraftState& state,
    double dtSeconds
) {

    // Disengaged autopilot produces no control request.
    if (!engaged_) {
        return 0.0;
    }

    // Integration requires a positive, finite timestep.
    if (!std::isfinite(dtSeconds) || dtSeconds <= 0.0) {
        throw std::invalid_argument(
            "Autopilot timestep must be finite and positive"
        );
    }

    // --------------------------------------------------------
    // 1. MEASURE THE CURRENT AIRCRAFT STATE
    // --------------------------------------------------------

    // Positive error means aircraft is BELOW target.
    const double error =
        altitudeErrorMeters(state);

    // Positive vertical speed means climbing.
    const double verticalSpeed =
        state.velocityMetersPerSecond.z;

    // Positive pitch rate means rotating nose-up.
    const double pitchRate =
        state.pitchRateRadiansPerSecond;

    // --------------------------------------------------------
    // 2. PROPORTIONAL CONTROL
    // --------------------------------------------------------

    // Respond to the current altitude error.
    //
    // Larger altitude difference -> stronger correction.
    const double proportional =
        kP * error;

    // --------------------------------------------------------
    // 3. DERIVATIVE CONTROL
    // --------------------------------------------------------

    // For a constant altitude target:
    //
    // d(error)/dt = -verticalSpeed
    //
    // This term slows the response as vertical speed grows.
    //
    // We use measured vertical speed instead of subtracting
    // consecutive altitude errors, which avoids derivative
    // kick when the target altitude changes.
    const double derivative =
        -kD * verticalSpeed;

    // --------------------------------------------------------
    // 4. ADDITIONAL PITCH-RATE DAMPING
    // --------------------------------------------------------

    // Oppose rapid nose-up or nose-down rotation.
    const double damping =
        -kPitchRate * pitchRate;

    // --------------------------------------------------------
    // 5. INTEGRAL CONTROL
    // --------------------------------------------------------

    // Integrate the altitude error over time.
    //
    // Integral error approximates:
    //
    // integral(error * dt)
    //
    // This helps eliminate persistent altitude offsets.
    //
    // First calculate a possible new integral value.
    const double candidateIntegral = std::clamp(
        accumulatedErrorMeterSeconds_
            + error * dtSeconds,
        -kIntegralLimit,
        kIntegralLimit
    );

    // Calculate the output we would get with that integral.
    const double candidateOutput =
        proportional
        + kI * candidateIntegral
        + derivative
        + damping;

    // --------------------------------------------------------
    // 6. ANTI-WINDUP
    // --------------------------------------------------------

    // Our normalized elevator request has limits of -1 to 1.
    //
    // If the controller is already requesting more than the
    // limit, don't keep integrating an error that pushes
    // even farther into saturation.
    //
    // This is a simple conditional-integration anti-windup
    // technique, not a complete actuator feedback model.

    const bool pushingHigh =
        candidateOutput > 1.0 && error > 0.0;

    const bool pushingLow =
        candidateOutput < -1.0 && error < 0.0;

    if (!pushingHigh && !pushingLow) {
        accumulatedErrorMeterSeconds_ = candidateIntegral;
    }

    // --------------------------------------------------------
    // 7. CALCULATE FINAL CONTROL REQUEST
    // --------------------------------------------------------

    const double output =
        proportional
        + kI * accumulatedErrorMeterSeconds_
        + derivative
        + damping;

    // Limit the requested elevator command.
    //
    // ControlSystem will convert this into the actual
    // elevator value and apply its movement-rate limit.
    return std::clamp(
        output,
        -1.0,
        1.0
    );
}

} // namespace flight
