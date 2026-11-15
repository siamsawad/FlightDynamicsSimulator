
#include "FlightSim/Aerodynamics.hpp"
#include "FlightSim/PhysicsEngine.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>

int main() {

    // Use types and functions from our flight namespace.
    using namespace flight;

    // Initialize the aircraft and pilot controls.
    AircraftState state;
    const AircraftParameters aircraft;
    Controls controls;

    // Create telemetry output file.
    std::ofstream csv("flight_telemetry.csv");

    if (!csv) {
        std::cerr << "Could not create flight_telemetry.csv\n";
        return 1;
    }

    // Create the CSV column headings.
    csv << "time_s,x_m,altitude_m,vx_mps,vz_mps,"
           "pitch_deg,alpha_deg\n";

    csv << std::fixed << std::setprecision(6);
    std::cout << std::fixed << std::setprecision(2);

    std::cout
        << "Time(s)  Altitude(m)  Speed(m/s)  Pitch(deg)\n";

    // Simulate 12 seconds at 120 physics steps per second.
    constexpr int totalSteps = 12 * 120;

    for (int step = 0; step <= totalSteps; ++step) {

        // Calculate the current simulation time.
        const double time = step * kTimeStep;

        // Apply the same elevator command as Version 1.
        controls.elevator =
            (time >= 2.0 && time < 2.5) ? 0.12 : 0.0;

        // Calculate aerodynamic forces and moment.
        const Loads loads =
            calculateLoads(state, aircraft, controls);

        // Write flight data to CSV.
        csv << time << ','
            << state.positionMeters.x << ','
            << state.positionMeters.z << ','
            << state.velocityMetersPerSecond.x << ','
            << state.velocityMetersPerSecond.z << ','
            << degrees(state.pitchRadians) << ','
            << degrees(loads.angleOfAttackRadians) << '\n';

        // Print results every simulated second.
        if (step % 120 == 0) {

            std::cout
                << std::setw(7) << time << "  "
                << std::setw(11) << state.positionMeters.z << "  "
                << std::setw(10) << loads.airspeedMetersPerSecond
                << "  "
                << std::setw(10) << degrees(state.pitchRadians)
                << '\n';
        }

        // Advance the aircraft by one fixed time step.
        if (step < totalSteps) {
            stepPhysics(state, aircraft, loads, kTimeStep);
        }
    }

    std::cout << "\nSaved flight_telemetry.csv\n";

    return 0;
}
