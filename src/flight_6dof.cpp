
#include "FlightSim/Dynamics6DOF.hpp"
#include "FlightSim/SimulationConfig.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

int main() {

    using namespace flight;
    using namespace flight::sixdof;

    // ========================================================
    // 1. USE THE EXISTING AIRCRAFT CONFIGURATION
    // ========================================================

    // This demo starts with default Version 8 settings.
    // The new physics engine accepts custom configuration
    // objects, but this console demo does not yet provide
    // an interactive configuration menu.
    const SimulationConfig config{};

    Model model;
    model.aircraft = config.aircraft;

    const Wind wind = makeWind(
        config.environment
    );

    State state = makeInitialState(config);

    // ========================================================
    // 2. CREATE 6-DOF TELEMETRY FILE
    // ========================================================

    std::ofstream csv(
        "flight_6dof_telemetry.csv"
    );

    if (!csv) {

        std::cerr
            << "Could not create "
            << "flight_6dof_telemetry.csv\n";

        return 1;
    }

    csv << std::fixed
        << std::setprecision(6);

    csv
        << "time_s,north_m,east_m,down_m,altitude_ft,"
        << "vn_mps,ve_mps,vd_mps,airspeed_kt,"
        << "roll_deg,pitch_deg,yaw_deg,"
        << "p_radps,q_radps,r_radps,"
        << "alpha_deg,beta_deg\n";

    // ========================================================
    // 3. PRINT TELEMETRY HEADER
    // ========================================================

    constexpr int steps = 24 * 120;

    std::cout
        << std::fixed
        << std::setprecision(2);

    std::cout
        << "Version 10: Six-degree-of-freedom flight demo\n"
        << "Coordinates: world NED, body FRD; "
        << "angles in degrees.\n"
        << "  t(s)  alt(ft)  speed(kt)  "
        << "roll(deg) pitch(deg)  yaw(deg)\n";

    // ========================================================
    // 4. FIXED-TIMESTEP SIMULATION
    // ========================================================

    for (int i = 0; i <= steps; ++i) {

        const double time = i * kTimeStep;

        Inputs input;

        // ----------------------------------------------------
        // THREE TEST MANEUVERS
        // ----------------------------------------------------

        // At 4 seconds: brief right-roll command.
        if (time >= 4.0 && time < 4.6) {
            input.aileron = 0.18;
        }

        // At 10 seconds: brief right-yaw command.
        if (time >= 10.0 && time < 10.6) {
            input.rudder = 0.15;
        }

        // At 16 seconds: brief nose-up command.
        if (time >= 16.0 && time < 16.6) {
            input.elevator = 0.25;
        }

        // Advance the physical state.
        if (i > 0) {

            step(
                state,
                model,
                input,
                wind,
                kTimeStep
            );
        }

        // ----------------------------------------------------
        // CALCULATE FLIGHT MEASUREMENTS
        // ----------------------------------------------------

        const EulerAngles angles = toEuler(
            state.bodyToWorld
        );

        const ForcesMoments forces =
            calculateForcesMoments(
                state,
                model,
                input,
                wind
            );

        // World NED uses negative Z for altitude.
        const double altitudeFt =
            -state.positionWorldMeters.z
            / kFeetToMeters;

        const double speedKt =
            forces.air.trueAirspeedMetersPerSecond
            / kKnotsToMetersPerSecond;

        // ----------------------------------------------------
        // RECORD TELEMETRY AT 10 HZ
        // ----------------------------------------------------

        if (i % 12 == 0) {

            csv
                << time << ','
                << state.positionWorldMeters.x << ','
                << state.positionWorldMeters.y << ','
                << state.positionWorldMeters.z << ','
                << altitudeFt << ','
                << state.velocityWorldMetersPerSecond.x << ','
                << state.velocityWorldMetersPerSecond.y << ','
                << state.velocityWorldMetersPerSecond.z << ','
                << speedKt << ','
                << degrees(angles.rollRadians) << ','
                << degrees(angles.pitchRadians) << ','
                << degrees(angles.yawRadians) << ','
                << state.angularVelocityBodyRadPerSecond.x << ','
                << state.angularVelocityBodyRadPerSecond.y << ','
                << state.angularVelocityBodyRadPerSecond.z << ','
                << degrees(
                    forces.air.angleOfAttackRadians
                ) << ','
                << degrees(
                    forces.air.sideslipRadians
                ) << '\n';
        }

        // ----------------------------------------------------
        // PRINT SUMMARY EVERY TWO SECONDS
        // ----------------------------------------------------

        if (i % 240 == 0) {

            std::cout
                << std::setw(6) << time
                << std::setw(9) << altitudeFt
                << std::setw(11) << speedKt
                << std::setw(11)
                   << degrees(angles.rollRadians)
                << std::setw(11)
                   << degrees(angles.pitchRadians)
                << std::setw(11)
                   << degrees(angles.yawRadians)
                << '\n';
        }

        // Detect numerical instability.
        if (
            !std::isfinite(altitudeFt)
            || !std::isfinite(speedKt)
        ) {

            std::cerr
                << "Nonfinite simulation state. Aborting.\n";

            return 2;
        }
    }

    std::cout
        << "Saved flight_6dof_telemetry.csv\n";

    return 0;
}

