
#include "FlightSim/Autopilot6DOF.hpp"
#include "FlightSim/Controls6DOF.hpp"
#include "FlightSim/Dynamics6DOF.hpp"
#include "FlightSim/SimulationConfig.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

int main() {

    using namespace flight;
    using namespace flight::sixdof;

    // ========================================================
    // 1. DEFINE FLIGHT SCENARIOS
    // ========================================================

    struct Scenario {

        const char* name;

        double altitudeChangeFeet;
        double headingChangeDegrees;
    };

    const Scenario scenarios[] = {

        {"level_hold", 0.0, 0.0},

        {"climb_100ft", 100.0, 0.0},

        {"turn_right_20deg", 0.0, 20.0},

        {"climb_and_turn", 100.0, 20.0}
    };

    // ========================================================
    // 2. CREATE TELEMETRY CSV
    // ========================================================

    std::ofstream csv(
        "flight_6dof_scenarios.csv"
    );

    if (!csv) {

        std::cerr
            << "Could not create scenario CSV file.\n";

        return 1;
    }

    csv << std::fixed
        << std::setprecision(6);

    csv
        << "scenario,time_s,altitude_ft,"
        << "target_altitude_ft,altitude_error_ft,"
        << "heading_deg,target_heading_deg,"
        << "heading_error_deg,airspeed_kts,"
        << "roll_deg,pitch_deg,"
        << "elevator,aileron,rudder\n";

    // ========================================================
    // 3. INITIALIZE AIRCRAFT MODEL
    // ========================================================

    const SimulationConfig config;

    Model model;
    model.aircraft = config.aircraft;

    const Wind wind =
        makeWind(config.environment);

    const Controls6DOF actuators;

    std::cout
        << std::fixed
        << std::setprecision(2);

    std::cout
        << "6-DOF autopilot flight scenarios\n"
        << "90 simulated seconds per scenario\n\n";

    // ========================================================
    // 4. RUN EACH SCENARIO
    // ========================================================

    for (const auto& scenario : scenarios) {

        // Restart from identical initial conditions.
        State state =
            makeInitialState(config);

        Inputs controls;

        Autopilot6DOF autopilot;

        // Engage at current altitude and heading.
        autopilot.engage(state);

        // Apply scenario-specific altitude target.
        autopilot.adjustTargetAltitudeMeters(
            scenario.altitudeChangeFeet *
            kFeetToMeters
        );

        // Apply scenario-specific heading target.
        autopilot.adjustTargetHeadingRadians(
            radians(
                scenario.headingChangeDegrees
            )
        );

        double maximumBankDegrees = 0.0;
        double minimumAirspeed = 1e9;

        constexpr int totalSteps = 90 * 120;

        // ----------------------------------------------------
        // A. RUN 90 SECONDS OF FLIGHT
        // ----------------------------------------------------

        for (int i = 0; i <= totalSteps; ++i) {

            if (i > 0) {

                // Calculate current aerodynamic data.
                const ForcesMoments current =
                    calculateForcesMoments(
                        state,
                        model,
                        controls,
                        wind
                    );

                // Autopilot calculates requested surfaces.
                const PilotAxes command =
                    autopilot.update(
                        state,
                        current.air,
                        model,
                        kTimeStep
                    );

                // Apply actuator movement-rate limits.
                actuators.update(
                    controls,
                    command,
                    kTimeStep
                );

                // Integrate complete 6-DOF physics.
                step(
                    state,
                    model,
                    controls,
                    wind,
                    kTimeStep
                );
            }

            // ------------------------------------------------
            // B. MEASURE FLIGHT PERFORMANCE
            // ------------------------------------------------

            const EulerAngles attitude =
                toEuler(state.bodyToWorld);

            const ForcesMoments loads =
                calculateForcesMoments(
                    state,
                    model,
                    controls,
                    wind
                );

            maximumBankDegrees = std::max(
                maximumBankDegrees,
                std::abs(
                    degrees(attitude.rollRadians)
                )
            );

            minimumAirspeed = std::min(
                minimumAirspeed,
                loads.air.trueAirspeedMetersPerSecond
            );

            // ------------------------------------------------
            // C. RECORD AT 10 SAMPLES PER SECOND
            // ------------------------------------------------

            if (i % 12 == 0) {

                const double headingError =
                    std::remainder(
                        autopilot.status()
                            .targetHeadingRadians -
                        attitude.yawRadians,
                        2.0 * kPi
                    );

                const double altitudeError =
                    autopilot.status()
                        .targetAltitudeMeters +
                    state.positionWorldMeters.z;

                csv
                    << scenario.name << ','
                    << i * kTimeStep << ','

                    << -state.positionWorldMeters.z /
                       kFeetToMeters << ','

                    << autopilot.status()
                           .targetAltitudeMeters /
                       kFeetToMeters << ','

                    << altitudeError /
                       kFeetToMeters << ','

                    << degrees(
                        attitude.yawRadians
                       ) << ','

                    << degrees(
                        autopilot.status()
                            .targetHeadingRadians
                       ) << ','

                    << degrees(headingError) << ','

                    << loads.air.trueAirspeedMetersPerSecond /
                       kKnotsToMetersPerSecond << ','

                    << degrees(
                        attitude.rollRadians
                       ) << ','

                    << degrees(
                        attitude.pitchRadians
                       ) << ','

                    << controls.elevator << ','
                    << controls.aileron << ','
                    << controls.rudder << '\n';
            }
        }

        // ----------------------------------------------------
        // D. PRINT FINAL SCENARIO RESULTS
        // ----------------------------------------------------

        const EulerAngles attitude =
            toEuler(state.bodyToWorld);

        const double altitudeError =
            (
                autopilot.status().targetAltitudeMeters +
                state.positionWorldMeters.z
            ) / kFeetToMeters;

        const double headingError = degrees(
            std::remainder(
                autopilot.status()
                    .targetHeadingRadians -
                attitude.yawRadians,
                2.0 * kPi
            )
        );

        std::cout
            << scenario.name
            << ": altitude error "
            << altitudeError << " ft"

            << ", heading error "
            << headingError << " deg"

            << ", peak bank "
            << maximumBankDegrees << " deg"

            << ", minimum airspeed "
            << minimumAirspeed /
               kKnotsToMetersPerSecond
            << " kt\n";
    }

    std::cout
        << "\nSaved flight_6dof_scenarios.csv\n";

    return 0;
}
