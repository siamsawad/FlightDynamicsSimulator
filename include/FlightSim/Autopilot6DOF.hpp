
#pragma once

#include "FlightSim/Controls6DOF.hpp"
#include "FlightSim/Dynamics6DOF.hpp"

namespace flight::sixdof {

// ============================================================
// AUTOPILOT STATUS
// ============================================================

// Information available to our graphical instruments
// and telemetry recorder.
struct Autopilot6DOFStatus {

    bool engaged = false;

    // Advisory conditions, not certified protections.
    bool lowAirspeed = false;
    bool highAngleOfAttack = false;

    // Commanded flight conditions.
    double targetAltitudeMeters = 1000.0;
    double targetHeadingRadians = 0.0;

    // Tracking errors.
    double altitudeErrorMeters = 0.0;
    double headingErrorRadians = 0.0;

    // Requested flight attitude.
    double desiredPitchRadians = 0.0;
    double desiredBankRadians = 0.0;

    // Approximate low-airspeed warning boundary.
    double minimumRecommendedAirspeedMetersPerSecond = 0.0;

    // Elevator, aileron, and rudder requests.
    PilotAxes requestedAxes{};
};

// ============================================================
// 6-DOF AUTOPILOT
// ============================================================
//
// Altitude hold:
//   altitude error -> desired pitch -> elevator
//
// Heading hold:
//   heading error -> desired bank -> aileron
//
// Rudder:
//   heading feedback + sideslip damping
//
// Output passes through our existing Controls6DOF.
//
// Engine throttle remains under manual control.
class Autopilot6DOF {
public:

    // Capture current altitude and heading.
    void engage(const State& state);

    // Return control to the pilot.
    void disengage();

    bool isEngaged() const {
        return status_.engaged;
    }

    // Altitude target, in internal SI units.
    void setTargetAltitudeMeters(double target);
    void adjustTargetAltitudeMeters(double delta);

    // Heading target, in radians.
    void setTargetHeadingRadians(double heading);
    void adjustTargetHeadingRadians(double delta);

    // Read-only status for HUD and telemetry.
    const Autopilot6DOFStatus& status() const {
        return status_;
    }

    // Calculate control requests once per physics step.
    PilotAxes update(
        const State& state,
        const AirData& air,
        const Model& model,
        double dtSeconds
    );

private:

    Autopilot6DOFStatus status_{};

    // Accumulated altitude error.
    double altitudeIntegralMeterSeconds_ = 0.0;
};

} // namespace flight::sixdof
