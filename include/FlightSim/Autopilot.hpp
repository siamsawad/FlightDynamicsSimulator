
#pragma once

#include "FlightSim/Aircraft.hpp"

namespace flight {

// ============================================================
// AUTOPILOT - ALTITUDE HOLD CONTROLLER
// ============================================================
//
// This controller produces a normalized elevator request:
//
// -1.0 = maximum nose-down request
//  0.0 = neutral
// +1.0 = maximum nose-up request
//
// The existing ControlSystem converts that request into
// an actual, rate-limited elevator command.
//
// The autopilot does NOT directly change aircraft position.
// ============================================================

class Autopilot {
public:

    // Engage altitude hold at the aircraft's CURRENT altitude.
    void engage(const AircraftState& state);

    // Return control to the pilot.
    void disengage();

    // Check whether the autopilot is currently engaged.
    bool isEngaged() const {
        return engaged_;
    }

    // Set a specific target altitude, in meters.
    void setTargetAltitudeMeters(double altitudeMeters);

    // Raise or lower the target by a specified amount.
    void adjustTargetAltitudeMeters(double deltaMeters);

    // Read the current target altitude.
    double targetAltitudeMeters() const {
        return targetAltitudeMeters_;
    }

    // Error = target altitude - actual altitude.
    double altitudeErrorMeters(
        const AircraftState& state
    ) const;

    // Calculate the elevator request.
    //
    // Call once every fixed physics update.
    // Returns a normalized command from -1 to +1.
    double update(
        const AircraftState& state,
        double dtSeconds
    );

private:

    // Autopilot starts disabled.
    bool engaged_ = false;

    // Default target before engagement.
    double targetAltitudeMeters_ = 1000.0;

    // Accumulated altitude error (meter-seconds).
    double accumulatedErrorMeterSeconds_ = 0.0;

    // --------------------------------------------------------
    // CONTROLLER TUNING PARAMETERS
    // --------------------------------------------------------
    //
    // These initial gains are tuned for our educational
    // Version 5 aircraft model, not a real aircraft.
    //
    // kP: Response to altitude error.
    // kI: Response to accumulated error.
    // kD: Response to vertical speed.
    // kPitchRate: Additional pitch-rate damping.

    static constexpr double kP = 0.035;

    static constexpr double kI = 0.002;

    static constexpr double kD = 0.20;

    static constexpr double kPitchRate = 1.0;

    // Prevent unlimited accumulation of altitude error.
    static constexpr double kIntegralLimit = 100.0;
};

} // namespace flight
