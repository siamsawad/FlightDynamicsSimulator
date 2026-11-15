
#include "FlightSim/Aerodynamics.hpp"
#include "FlightSim/Autopilot.hpp"
#include "FlightSim/Telemetry.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

namespace {

bool near(double actual, double expected) {
    return std::abs(actual - expected) < 1e-8;
}

bool check(bool condition, const char* message) {

    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }

    return true;
}

} // namespace

int main() {

    using namespace flight;

    const std::string filename =
        "test_telemetry_output.csv";

    bool passed = true;

    // Use an inner scope so the CSV file closes before
    // we attempt to read and delete it.
    {
        TelemetryRecorder telemetry(filename);

        AircraftState state;
        const AircraftParameters aircraft;
        Controls controls;

        Autopilot autopilot;

        passed &= check(
            telemetry.isLogging(),
            "CSV file opens"
        );

        // Engage at 1000 m, then command 1010 m.
        autopilot.engage(state);
        autopilot.setTargetAltitudeMeters(1010.0);

        // First interval: aircraft is 10 m below target.
        telemetry.updateTracking(
            state, autopilot, 0.5
        );

        // Second interval: aircraft is 5 m below target.
        state.positionMeters.z = 1005.0;

        telemetry.updateTracking(
            state, autopilot, 0.5
        );

        // Expected time-weighted mean:
        //
        // (10 * 0.5 + 5 * 0.5) / 1.0 = 7.5 m.
        const auto& stats = telemetry.statistics();

        passed &= check(
            near(stats.engagedSeconds, 1.0),
            "Autopilot engaged duration"
        );

        passed &= check(
            near(stats.meanAbsoluteErrorMeters(), 7.5),
            "Mean absolute altitude error"
        );

        passed &= check(
            near(stats.maximumAbsoluteErrorMeters, 10.0),
            "Maximum absolute altitude error"
        );

        // Record two samples.
        const Loads loads =
            calculateLoads(state, aircraft, controls);

        telemetry.recordSample(
            0.0, state, loads, controls, autopilot
        );

        telemetry.recordSample(
            0.1, state, loads, controls, autopilot
        );

        passed &= check(
            telemetry.history().size() == 2,
            "Flight-history storage"
        );

        // Reset to a new simulation run.
        telemetry.startNewRun();

        passed &= check(
            telemetry.history().empty() &&
            near(
                telemetry.statistics().engagedSeconds,
                0.0
            ),
            "Reset history and statistics"
        );

        // Save another sample under run ID 2.
        autopilot.disengage();

        telemetry.recordSample(
            0.0, state, loads, controls, autopilot
        );
    }

    // The telemetry file is now closed.
    std::ifstream csv(filename);

    passed &= check(
        csv.is_open(),
        "Recorded CSV can be reopened"
    );

    std::string row;
    int rowCount = 0;
    bool foundSecondRun = false;

    while (std::getline(csv, row)) {

        ++rowCount;

        if (row.rfind("2,", 0) == 0) {
            foundSecondRun = true;
        }
    }

    // One header + three data rows.
    passed &= check(
        rowCount == 4,
        "Correct CSV row count"
    );

    passed &= check(
        foundSecondRun,
        "Multiple flight run IDs"
    );

    csv.close();

    // Delete the temporary testing file.
    std::remove(filename.c_str());

    if (!passed) {
        return 1;
    }

    std::cout
        << "PASS: All telemetry tests passed.\n";

    return 0;
}
