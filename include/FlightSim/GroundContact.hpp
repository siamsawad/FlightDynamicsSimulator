#pragma once

#include "FlightSim/Aircraft.hpp"

namespace flight {

// Results of checking the aircraft against the runway.
struct GroundContactReport {

    bool onGround = false;
    bool touchedDown = false;
    bool hardLanding = false;

    double impactSpeedMetersPerSecond = 0.0;

    double normalForceNewtons = 0.0;
    double rollingResistanceNewtons = 0.0;
};

// Simple flat-runway interaction for our 2D aircraft.
//
// Current limitations:
// - Runway at sea level (0 meters).
// - Aircraft represented by one reference point.
// - No landing gear, suspension, tire grip, or brakes.
class GroundContact {
public:

    // Run this AFTER PhysicsEngine::stepPhysics().
    GroundContactReport resolve(
        AircraftState& state,
        const AircraftParameters& aircraft,
        const Loads& loads,
        double dtSeconds
    );

    // Reset contact history before a new flight.
    void reset() {
        wasOnGround_ = false;
    }

private:

    bool wasOnGround_ = false;

    static constexpr double kRunwayAltitudeMeters = 0.0;

    // Simplified rolling-resistance coefficient.
    static constexpr double kRollingResistanceCoefficient = 0.02;
};

} // namespace flight