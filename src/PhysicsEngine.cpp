
#include "FlightSim/PhysicsEngine.hpp"

namespace flight {

void stepPhysics(AircraftState& state,
                 const AircraftParameters& aircraft,
                 const Loads& loads,
                 double dt) {

    // Calculate linear acceleration: a = F / m.
    const double ax =
        loads.netForceNewtons.x / aircraft.massKg;

    const double az =
        loads.netForceNewtons.z / aircraft.massKg;

    // Angular acceleration = pitching moment / inertia.
    const double pitchAcceleration =
        loads.pitchMomentNoseUpNm /
        aircraft.pitchInertiaKgM2;

    // Update horizontal and vertical velocities.
    state.velocityMetersPerSecond.x += ax * dt;
    state.velocityMetersPerSecond.z += az * dt;

    // Update pitch rotation rate.
    state.pitchRateRadiansPerSecond +=
        pitchAcceleration * dt;

    // Update position using the new velocities.
    state.positionMeters.x +=
        state.velocityMetersPerSecond.x * dt;

    state.positionMeters.z +=
        state.velocityMetersPerSecond.z * dt;

    // Update aircraft pitch angle.
    state.pitchRadians +=
        state.pitchRateRadiansPerSecond * dt;
}

} // namespace flight
