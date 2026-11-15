
#pragma once

#include "FlightSim/Aircraft.hpp"
#include "FlightSim/Autopilot.hpp"

#include <cstddef>
#include <deque>
#include <fstream>
#include <string>

namespace flight {

// Shared conversions between simulation and display units.
// Physics remains in SI units.
inline constexpr double kMetersToFeet = 3.280839895013123;
inline constexpr double kMetersPerSecondToKnots = 1.943844492;
inline constexpr double kMetersPerSecondToFeetPerMinute =
    196.8503937;

// One sample of the aircraft's flight history.
struct TelemetrySample {
    double timeSeconds = 0.0;
    double altitudeMeters = 0.0;

    bool autopilotEngaged = false;
    double targetAltitudeMeters = 0.0;
};

// Measures altitude-tracking error while autopilot is ON.
struct TrackingStatistics {

    // Total simulated seconds with autopilot engaged.
    double engagedSeconds = 0.0;

    // Integral of absolute altitude error over time.
    double absoluteErrorMeterSeconds = 0.0;

    // Largest absolute altitude error recorded.
    double maximumAbsoluteErrorMeters = 0.0;

    // Time-weighted mean absolute altitude error.
    double meanAbsoluteErrorMeters() const {

        if (engagedSeconds <= 0.0) {
            return 0.0;
        }

        return absoluteErrorMeterSeconds / engagedSeconds;
    }
};

// ============================================================
// TELEMETRY RECORDER
// ============================================================
//
// Responsibilities:
// 1. Write flight data to CSV.
// 2. Store a rolling 60-second history for graphs.
// 3. Calculate autopilot tracking statistics.
//
// No SFML dependency is needed here.
class TelemetryRecorder {
public:

    // Open a CSV file and write its header.
    explicit TelemetryRecorder(const std::string& filename);

    // Accumulate tracking performance once per physics step.
    void updateTracking(
        const AircraftState& state,
        const Autopilot& autopilot,
        double dtSeconds
    );

    // Record one flight measurement and update graph history.
    void recordSample(
        double timeSeconds,
        const AircraftState& state,
        const Loads& loads,
        const Controls& controls,
        const Autopilot& autopilot
    );

    // Called when the user resets the aircraft.
    // Clears statistics and starts a new numbered run.
    void startNewRun();

    // Recent measurements for the graph.
    const std::deque<TelemetrySample>& history() const {
        return history_;
    }

    // Autopilot performance statistics.
    const TrackingStatistics& statistics() const {
        return statistics_;
    }

    // Check whether the CSV file opened successfully.
    bool isLogging() const {
        return csv_.is_open();
    }

private:

    std::ofstream csv_;

    // Limited in-memory history for real-time graphing.
    std::deque<TelemetrySample> history_;

    TrackingStatistics statistics_;

    // Each reset starts another numbered simulation run.
    int runId_ = 1;

    // Used for occasional disk flushing.
    std::size_t samplesWritten_ = 0;

    static constexpr double kHistorySeconds = 60.0;
};

} // namespace flight
