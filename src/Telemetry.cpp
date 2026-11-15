
#include "FlightSim/Telemetry.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>

namespace flight {

// ============================================================
// CONSTRUCTOR
// ============================================================

TelemetryRecorder::TelemetryRecorder(
    const std::string& filename
) {

    // Open the CSV file.
    //
    // Starting the graphical application creates a new
    // telemetry file, replacing a file of the same name
    // from a previous application launch.
    csv_.open(filename, std::ios::out | std::ios::trunc);

    if (!csv_) {
        std::cerr
            << "Warning: Could not open telemetry file: "
            << filename << '\n';
        return;
    }

    // Use six digits after the decimal point.
    csv_ << std::fixed << std::setprecision(6);

    // Every heading includes a name and, when applicable,
    // a unit. This is important for engineering datasets.
    csv_
        << "run_id,time_s,x_m,altitude_m,altitude_ft,"
        << "airspeed_kts,vertical_speed_ft_min,"
        << "pitch_deg,aoa_deg,throttle_fraction,"
        << "elevator_command,autopilot_engaged,"
        << "target_altitude_ft,altitude_error_ft,"
        << "stall_model_active\n";
}

// ============================================================
// AUTOPILOT TRACKING METRICS
// ============================================================

void TelemetryRecorder::updateTracking(
    const AircraftState& state,
    const Autopilot& autopilot,
    double dtSeconds
) {

    // Only measure altitude tracking while AP is engaged.
    if (!autopilot.isEngaged()) {
        return;
    }

    if (!std::isfinite(dtSeconds) || dtSeconds <= 0.0) {
        return;
    }

    // Absolute difference between target and actual altitude.
    const double absoluteError = std::abs(
        autopilot.altitudeErrorMeters(state)
    );

    // Add one physics step to the tracking duration.
    statistics_.engagedSeconds += dtSeconds;

    // Integrate absolute error over this time step.
    statistics_.absoluteErrorMeterSeconds +=
        absoluteError * dtSeconds;

    // Record the largest error observed so far.
    statistics_.maximumAbsoluteErrorMeters = std::max(
        statistics_.maximumAbsoluteErrorMeters,
        absoluteError
    );
}

// ============================================================
// RECORD A FLIGHT SAMPLE
// ============================================================

void TelemetryRecorder::recordSample(
    double timeSeconds,
    const AircraftState& state,
    const Loads& loads,
    const Controls& controls,
    const Autopilot& autopilot
) {

    // --------------------------------------------------------
    // 1. STORE DATA FOR THE GRAPH
    // --------------------------------------------------------

    TelemetrySample sample;

    sample.timeSeconds = timeSeconds;
    sample.altitudeMeters = state.positionMeters.z;

    sample.autopilotEngaged = autopilot.isEngaged();

    if (autopilot.isEngaged()) {
        sample.targetAltitudeMeters =
            autopilot.targetAltitudeMeters();
    }

    history_.push_back(sample);

    // Remove samples older than the rolling 60-second window.
    while (!history_.empty() &&
           history_.front().timeSeconds <
               timeSeconds - kHistorySeconds) {

        history_.pop_front();
    }

    // --------------------------------------------------------
    // 2. WRITE THE CSV ROW
    // --------------------------------------------------------

    // Graph history still works if disk logging failed.
    if (!csv_.is_open()) {
        return;
    }

    // Convert internal SI values to aviation units.
    const double altitudeFeet =
        state.positionMeters.z * kMetersToFeet;

    const double speedKnots =
        loads.airspeedMetersPerSecond *
        kMetersPerSecondToKnots;

    const double verticalSpeedFeetPerMinute =
        state.velocityMetersPerSecond.z *
        kMetersPerSecondToFeetPerMinute;

    csv_
        << runId_ << ','
        << timeSeconds << ','
        << state.positionMeters.x << ','
        << state.positionMeters.z << ','
        << altitudeFeet << ','
        << speedKnots << ','
        << verticalSpeedFeetPerMinute << ','
        << degrees(state.pitchRadians) << ','
        << degrees(loads.angleOfAttackRadians) << ','
        << controls.throttle << ','
        << controls.elevator << ','
        << (autopilot.isEngaged() ? 1 : 0) << ',';

    // When AP is disengaged, these fields are intentionally
    // empty, rather than pretending a target exists.
    if (autopilot.isEngaged()) {

        csv_
            << autopilot.targetAltitudeMeters() *
                   kMetersToFeet
            << ','
            << autopilot.altitudeErrorMeters(state) *
                   kMetersToFeet
            << ',';

    } else {

        // Empty target-altitude and error columns.
        csv_ << ",,";
    }

    csv_ << (loads.isStalled ? 1 : 0) << '\n';

    ++samplesWritten_;

    // Flush approximately once a second when sampling
    // at the planned 10 Hz rate.
    if (samplesWritten_ % 10 == 0) {
        csv_.flush();
    }
}

// ============================================================
// RESET FLIGHT HISTORY
// ============================================================

void TelemetryRecorder::startNewRun() {

    ++runId_;

    // Clear the on-screen history and AP metrics.
    history_.clear();
    statistics_ = TrackingStatistics{};

    // Do not truncate the CSV file here.
    // Previous runs remain in the current session's file.
}

} // namespace flight
