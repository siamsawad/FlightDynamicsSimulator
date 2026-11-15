
#pragma once

// ============================================================
// AIRCRAFT DATA STRUCTURES - VERSION 5
// ============================================================
//
// Shared aircraft state, parameters and aerodynamic results.
//
// SI units: meters, seconds, kilograms, newtons, radians.
//
// World coordinates:
// +X = forward
// +Y = left
// +Z = upward
//
// Positive pitch is nose-up.
// Our current dynamics operate in X/Z only.
// ============================================================

namespace flight {

// Mathematical and simulation constants.
inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kGravity = 9.81;

// Reference sea-level air density (kg/m^3).
// The aerodynamic model will now calculate actual density
// based on altitude instead of always using this value.
inline constexpr double kAirDensity = 1.225;

// Physics engine runs at 120 updates per simulated second.
inline constexpr double kTimeStep = 1.0 / 120.0;

// Convert degrees to radians for physics calculations.
inline double radians(double valueDegrees) {
    return valueDegrees * kPi / 180.0;
}

// Convert radians to degrees for instruments and telemetry.
inline double degrees(double valueRadians) {
    return valueRadians * 180.0 / kPi;
}

// ============================================================
// VECTOR3
// ============================================================

// Represents a quantity in three-dimensional space.
//
// Examples:
// Position: (x, y, z) in meters.
// Velocity: (vx, vy, vz) in meters per second.
// Force:    (Fx, Fy, Fz) in newtons.
//
// We currently use X and Z. Keeping Y available supports
// our planned future 6-DOF expansion.
struct Vector3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// ============================================================
// AIRCRAFT STATE
// ============================================================

// Quantities that change during the flight simulation.
struct AircraftState {

    // Starting position: 1,000 meters above sea level.
    Vector3 positionMeters{0.0, 0.0, 1000.0};

    // Starting velocity: 50 m/s horizontally.
    Vector3 velocityMetersPerSecond{50.0, 0.0, 0.0};

    // Starting orientation.
    double pitchRadians = radians(1.75);

    // Starting angular velocity.
    double pitchRateRadiansPerSecond = 0.0;
};

// ============================================================
// AIRCRAFT PARAMETERS
// ============================================================

// Defines the physical characteristics of our aircraft.
//
// These are illustrative engineering parameters,
// NOT measured values for a real aircraft.
struct AircraftParameters {

    // -------------------------------
    // MASS AND ROTATIONAL INERTIA
    // -------------------------------

    double massKg = 1000.0;

    // Resistance to pitch acceleration.
    double pitchInertiaKgM2 = 2500.0;

    // -------------------------------
    // WING GEOMETRY
    // -------------------------------

    double wingAreaM2 = 16.0;

    // Mean aerodynamic chord.
    double meanChordMeters = 1.5;

    // Wingspan used to calculate aspect ratio.
    double wingSpanMeters = 10.6;

    // Efficiency factor used in induced drag.
    // Typical simplified models use a value below 1.
    double oswaldEfficiency = 0.80;

    // -------------------------------
    // ENGINE
    // -------------------------------

    double maxThrustNewtons = 3500.0;

    // -------------------------------
    // LIFT AND DRAG COEFFICIENTS
    // -------------------------------

    // Baseline drag coefficient.
    double zeroLiftDragCoefficient = 0.035;

    // Lift coefficient at zero angle of attack.
    //
    // Slightly adjusted from Version 4 so our existing
    // initial state remains close to level flight
    // after introducing altitude-dependent air density.
    double liftCoefficientAtZeroAoA = 0.289;

    // Change in lift coefficient per radian of AoA.
    double liftCurveSlopePerRadian = 5.0;

    // Nominal critical angle of attack.
    double stallAngleRadians = radians(15.0);

    // -------------------------------
    // LONGITUDINAL STABILITY
    // -------------------------------

    // Pitching moment coefficient at zero AoA.
    double pitchMomentAtZeroAoA = 0.018;

    // Negative slope provides restoring pitch moment.
    double pitchStabilityPerRadian = -0.60;

    // Elevator effectiveness for pitching moments.
    double elevatorMomentEffectiveness = 0.35;

    // Negative value opposes pitch rotation.
    double pitchDampingCoefficient = -8.0;
};

// ============================================================
// PILOT CONTROLS
// ============================================================

// These are the ACTUAL controls applied to aerodynamics.
//
// Version 4's ControlSystem smoothly moves the elevator
// toward the pilot's requested value.
struct Controls {

    // 0.0 = no throttle
    // 1.0 = maximum throttle
    double throttle = 0.30;

    // Positive = nose-up command.
    // Negative = nose-down command.
    double elevator = 0.0;
};

// ============================================================
// AERODYNAMIC LOADS AND MEASUREMENTS
// ============================================================

// Stores net forces, pitching moment, and diagnostics.
//
// Gravity is already included in netForceNewtons.
// The physics engine must not add gravity again.
struct Loads {

    // Combined external forces.
    Vector3 netForceNewtons{};

    // Net aerodynamic pitching moment.
    double pitchMomentNoseUpNm = 0.0;

    // Existing flight measurements.
    double airspeedMetersPerSecond = 0.0;
    double angleOfAttackRadians = 0.0;

    // New atmospheric measurements.
    double airDensityKgPerCubicMeter = 0.0;
    double dynamicPressurePascals = 0.0;

    // Individual aerodynamic and engine forces.
    double liftNewtons = 0.0;
    double dragNewtons = 0.0;
    double thrustNewtons = 0.0;

    // Coefficients for flight analysis.
    double liftCoefficient = 0.0;
    double dragCoefficient = 0.0;

    // Whether AoA exceeds the nominal stall boundary.
    bool isStalled = false;
};

} // namespace flight
