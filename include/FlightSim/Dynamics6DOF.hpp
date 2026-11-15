
#pragma once

#include "FlightSim/Aircraft.hpp"
#include "FlightSim/Environment.hpp"

namespace flight {

// Forward declaration of our existing configuration.
struct SimulationConfig;

namespace sixdof {

// ============================================================
// THREE-DIMENSIONAL VECTOR
// ============================================================
//
// World coordinates: NED (North, East, Down).
// Body coordinates:  FRD (Forward, Right, Down).
//
// The meaning depends on the variable name.
// Do not mix world and body vectors without rotation.
struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

Vec3 operator+(Vec3 a, Vec3 b);
Vec3 operator-(Vec3 a, Vec3 b);
Vec3 operator*(Vec3 v, double scalar);
Vec3 operator/(Vec3 v, double scalar);

double dot(Vec3 a, Vec3 b);
Vec3 cross(Vec3 a, Vec3 b);
double magnitude(Vec3 v);

// ============================================================
// QUATERNION
// ============================================================
//
// Represents orientation from BODY coordinates
// to WORLD coordinates.
//
// Component order: (w, x, y, z).
//
// We integrate orientation as a quaternion instead
// of integrating Euler angles directly.
struct Quaternion {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    static Quaternion fromEuler(
        double roll,
        double pitch,
        double yaw
    );

    Quaternion normalized() const;
};

Quaternion operator*(Quaternion a, Quaternion b);

Vec3 rotateBodyToWorld(
    Quaternion orientation,
    Vec3 bodyVector
);

Vec3 rotateWorldToBody(
    Quaternion orientation,
    Vec3 worldVector
);

// Convenient representation for instruments.
// These are derived from the quaternion.
struct EulerAngles {
    double rollRadians = 0.0;
    double pitchRadians = 0.0;
    double yawRadians = 0.0;
};

EulerAngles toEuler(Quaternion orientation);

// ============================================================
// SIX-DEGREE-OF-FREEDOM AIRCRAFT STATE
// ============================================================

struct State {

    // World NED position.
    //
    // Negative Z means altitude above sea level.
    // -1000 meters corresponds to +1000 m altitude.
    Vec3 positionWorldMeters{0.0, 0.0, -1000.0};

    // Velocity relative to the ground, in world NED.
    Vec3 velocityWorldMetersPerSecond{50.0, 0.0, 0.0};

    // Orientation of the aircraft body.
    Quaternion bodyToWorld{};

    // Angular rates in body coordinates:
    // x = roll rate p
    // y = pitch rate q
    // z = yaw rate r
    Vec3 angularVelocityBodyRadPerSecond{};
};

// ============================================================
// SIX-DOF PILOT INPUTS
// ============================================================
//
// Inputs are normalized requests.
//
// Throttle: 0 to 1.
// Control surfaces: -1 to +1.
//
// Elevator +/-1 represents the same +/-0.12
// aerodynamic command range as our original simulator.
struct Inputs {
    double throttle = 0.30;
    double elevator = 0.0;
    double aileron = 0.0;
    double rudder = 0.0;
};

// ============================================================
// AIRCRAFT MODEL
// ============================================================

struct Model {

    // Reuse the physical and aerodynamic parameters
    // developed in our previous versions.
    AircraftParameters aircraft{};

    // Additional moments of inertia.
    double rollInertiaKgM2 = 1300.0;
    double yawInertiaKgM2 = 3000.0;

    // Lateral and directional aerodynamic coefficients.
    //
    // These are illustrative coefficients,
    // not measured data from a particular airplane.
    double sideForceBeta = -0.70;

    double rollMomentBeta = -0.08;
    double rollMomentAileron = 0.10;
    double rollDamping = -0.45;

    double yawMomentBeta = 0.14;
    double yawMomentRudder = 0.12;
    double yawDamping = -0.32;
};

// ============================================================
// THREE-DIMENSIONAL WIND
// ============================================================

struct Wind {

    // Wind velocity in world NED coordinates.
    Vec3 velocityWorldMetersPerSecond{};

    // Multiplier applied to standard atmospheric density.
    double densityMultiplier = 1.0;
};

// ============================================================
// AERODYNAMIC MEASUREMENTS
// ============================================================

struct AirData {

    double trueAirspeedMetersPerSecond = 0.0;

    double angleOfAttackRadians = 0.0;
    double sideslipRadians = 0.0;

    double densityKgPerCubicMeter = 0.0;
    double dynamicPressurePascals = 0.0;

    double liftNewtons = 0.0;
    double dragNewtons = 0.0;
    double sideForceNewtons = 0.0;

    bool nominalStall = false;
};

// ============================================================
// FORCES AND MOMENTS
// ============================================================

struct ForcesMoments {

    // Aerodynamic and engine forces, in BODY FRD.
    Vec3 aerodynamicAndThrustForceBodyNewtons{};

    // All external forces, in WORLD NED.
    // This includes gravity.
    Vec3 netForceWorldNewtons{};

    // Body moments: roll, pitch, yaw.
    Vec3 momentBodyNm{};

    AirData air{};
};

// Calculate forces and moments without changing state.
ForcesMoments calculateForcesMoments(
    const State& state,
    const Model& model,
    const Inputs& inputs,
    const Wind& wind = Wind{}
);

// Advance the complete rigid-body state.
//
// Includes:
// - 3D translation
// - Rotational inertia and gyroscopic coupling
// - Quaternion orientation integration
void step(
    State& state,
    const Model& model,
    const Inputs& inputs,
    const Wind& wind,
    double dtSeconds
);

// Convert existing Version 8 configuration to 6-DOF.
State makeInitialState(
    const SimulationConfig& config
);

Wind makeWind(
    const Environment& environment
);

} // namespace sixdof
} // namespace flight
