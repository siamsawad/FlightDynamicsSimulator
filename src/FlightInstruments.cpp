
#include "FlightSim/FlightInstruments.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace flight {

namespace {

const sf::Color kWhite(244, 249, 255);
const sf::Color kMuted(165, 193, 214);
const sf::Color kCyan(83, 208, 245);
const sf::Color kGreen(104, 233, 166);

// Format a number for the display.
std::string number(double value, int decimals = 1) {

    std::ostringstream stream;

    stream << std::fixed
           << std::setprecision(decimals)
           << value;

    return stream.str();
}

// Draw a panel with a dark background.
void panel(
    sf::RenderWindow& window,
    float x,
    float y,
    float width,
    float height
) {

    sf::RectangleShape background({width, height});

    background.setPosition({x, y});
    background.setFillColor(sf::Color(14, 29, 47, 235));
    background.setOutlineColor(sf::Color(93, 135, 167));
    background.setOutlineThickness(1.0f);

    window.draw(background);
}

// Draw one line using two SFML vertices.
void line(
    sf::RenderWindow& window,
    sf::Vector2f start,
    sf::Vector2f end,
    sf::Color color
) {

    const sf::Vertex vertices[] = {
        {start, color},
        {end, color}
    };

    window.draw(
        vertices,
        2,
        sf::PrimitiveType::Lines
    );
}

} // anonymous namespace

// ============================================================
// CONSTRUCTOR
// ============================================================

FlightInstruments::FlightInstruments() {

    fontLoaded_ =
        font_.openFromFile("C:/Windows/Fonts/segoeui.ttf");

    if (!fontLoaded_) {
        fontLoaded_ =
            font_.openFromFile("C:/Windows/Fonts/arial.ttf");
    }
}

// ============================================================
// TEXT HELPER
// ============================================================

void FlightInstruments::drawText(
    sf::RenderWindow& window,
    const std::string& message,
    unsigned int size,
    sf::Vector2f position,
    sf::Color color
) const {

    if (!fontLoaded_) {
        return;
    }

    sf::Text text(font_, message, size);

    text.setPosition(position);
    text.setFillColor(color);

    window.draw(text);
}

// ============================================================
// PITCH ATTITUDE INDICATOR
// ============================================================

void FlightInstruments::drawAttitudeIndicator(
    sf::RenderWindow& window,
    const AircraftState& state
) const {

    // Place the instrument below the existing
    // altitude and pitch panels on the right.
    const float x =
        static_cast<float>(window.getSize().x) - 255.0f;

    constexpr float y = 334.0f;
    constexpr float width = 235.0f;
    constexpr float height = 232.0f;

    panel(window, x, y, width, height);

    drawText(
        window,
        "ATTITUDE / PITCH",
        15,
        {x + 12.0f, y + 8.0f},
        kWhite
    );

    // Dimensions of the artificial-horizon display.
    const float left = x + 17.0f;
    const float top = y + 39.0f;
    const float displayWidth = 201.0f;
    const float displayHeight = 127.0f;

    const float bottom = top + displayHeight;
    const float centerY = top + displayHeight / 2.0f;

    // Convert the aircraft's actual pitch to degrees.
    const double pitchDegrees =
        degrees(state.pitchRadians);

    // Positive nose-up pitch makes the horizon move DOWN
    // relative to the fixed aircraft reference symbol.
    //
    // This is a pitch-only display, not a 3D horizon.
    constexpr float pixelsPerDegree = 2.2f;

    const float horizon = std::clamp(
        centerY +
            static_cast<float>(pitchDegrees) *
            pixelsPerDegree,
        top,
        bottom
    );

    // The portion above the horizon represents sky.
    sf::RectangleShape sky({
        displayWidth,
        horizon - top
    });

    sky.setPosition({left, top});
    sky.setFillColor(sf::Color(65, 132, 194));
    window.draw(sky);

    // The portion below the horizon represents ground.
    sf::RectangleShape ground({
        displayWidth,
        bottom - horizon
    });

    ground.setPosition({left, horizon});
    ground.setFillColor(sf::Color(154, 104, 71));
    window.draw(ground);

    // Draw the horizon boundary.
    line(
        window,
        {left, horizon},
        {left + displayWidth, horizon},
        sf::Color(243, 245, 242)
    );

    // Stationary aircraft reference symbol.
    const float cx = left + displayWidth / 2.0f;
    const float cy = centerY;

    const sf::Color reference(255, 218, 91);

    line(
        window,
        {cx - 65.0f, cy},
        {cx - 15.0f, cy},
        reference
    );

    line(
        window,
        {cx + 15.0f, cy},
        {cx + 65.0f, cy},
        reference
    );

    line(
        window,
        {cx - 15.0f, cy},
        {cx, cy + 7.0f},
        reference
    );

    line(
        window,
        {cx, cy + 7.0f},
        {cx + 15.0f, cy},
        reference
    );

    // Draw a thin border around the attitude display.
    sf::RectangleShape frame({
        displayWidth,
        displayHeight
    });

    frame.setPosition({left, top});
    frame.setFillColor(sf::Color::Transparent);
    frame.setOutlineColor(sf::Color(155, 185, 209));
    frame.setOutlineThickness(1.0f);

    window.draw(frame);

    // Show the numerical pitch value.
    drawText(
        window,
        "PITCH: " + number(pitchDegrees, 1) + " deg",
        16,
        {x + 16.0f, y + 174.0f},
        kWhite
    );

    // Be explicit about the current simulation limitation.
    drawText(
        window,
        "ROLL / YAW: NOT MODELED (2D)",
        11,
        {x + 16.0f, y + 203.0f},
        kMuted
    );
}

// ============================================================
// ALTITUDE HISTORY GRAPH
// ============================================================

void FlightInstruments::drawAltitudeHistory(
    sf::RenderWindow& window,
    const TelemetryRecorder& telemetry
) const {

    // Position the chart below the aircraft.
    constexpr float x = 338.0f;
    constexpr float y = 440.0f;
    constexpr float width = 580.0f;
    constexpr float height = 192.0f;

    panel(window, x, y, width, height);

    drawText(
        window,
        "ALTITUDE HISTORY - LAST 60 SECONDS",
        15,
        {x + 13.0f, y + 7.0f},
        kWhite
    );

    const auto& samples = telemetry.history();
    const auto& stats = telemetry.statistics();

    // Display performance statistics for the current run.
    if (stats.engagedSeconds > 0.0) {

        drawText(
            window,
            "AP MEAN ERROR: " +
                number(
                    stats.meanAbsoluteErrorMeters() *
                    kMetersToFeet, 1
                ) +
                " ft   MAX ERROR: " +
                number(
                    stats.maximumAbsoluteErrorMeters *
                    kMetersToFeet, 1
                ) + " ft",
            12,
            {x + 13.0f, y + 31.0f},
            kGreen
        );

    } else {

        drawText(
            window,
            "Engage autopilot to measure tracking performance",
            12,
            {x + 13.0f, y + 31.0f},
            kMuted
        );
    }

    if (samples.empty()) {
        return;
    }

    // Plot area inside the panel.
    const float plotLeft = x + 17.0f;
    const float plotTop = y + 61.0f;
    const float plotWidth = width - 34.0f;
    const float plotHeight = 94.0f;

    // Determine the time range.
    const double latestTime = samples.back().timeSeconds;
    const double firstTime = std::max(
        0.0,
        latestTime - 60.0
    );

    // Always show a 60-second horizontal time scale.
    constexpr double timeRange = 60.0;

    // Find altitude limits for vertical graph scaling.
    double minimumAltitude =
        std::numeric_limits<double>::infinity();

    double maximumAltitude =
        -std::numeric_limits<double>::infinity();

    for (const auto& sample : samples) {

        minimumAltitude = std::min(
            minimumAltitude,
            sample.altitudeMeters
        );

        maximumAltitude = std::max(
            maximumAltitude,
            sample.altitudeMeters
        );

        // Include autopilot target when it exists.
        if (sample.autopilotEngaged) {

            minimumAltitude = std::min(
                minimumAltitude,
                sample.targetAltitudeMeters
            );

            maximumAltitude = std::max(
                maximumAltitude,
                sample.targetAltitudeMeters
            );
        }
    }

    // Add vertical padding so the graph isn't squeezed.
    const double paddingMeters = std::max(
        5.0,
        (maximumAltitude - minimumAltitude) * 0.15
    );

    minimumAltitude -= paddingMeters;
    maximumAltitude += paddingMeters;

    const double altitudeRange =
        maximumAltitude - minimumAltitude;

    // Convert simulation values to screen positions.
    auto screenX = [&](double timeSeconds) {

        return plotLeft +
            static_cast<float>(
                (timeSeconds - firstTime) /
                timeRange
            ) * plotWidth;
    };

    auto screenY = [&](double altitudeMeters) {

        return plotTop +
            static_cast<float>(
                (maximumAltitude - altitudeMeters) /
                altitudeRange
            ) * plotHeight;
    };

    // --------------------------------------------------------
    // DRAW GRAPH GRID
    // --------------------------------------------------------

    for (int i = 0; i <= 4; ++i) {

        const float gridY =
            plotTop + plotHeight * i / 4.0f;

        line(
            window,
            {plotLeft, gridY},
            {plotLeft + plotWidth, gridY},
            sf::Color(67, 89, 110)
        );
    }

    // --------------------------------------------------------
    // DRAW ACTUAL ALTITUDE
    // --------------------------------------------------------

    sf::VertexArray altitudeLine(
        sf::PrimitiveType::LineStrip
    );

    for (const auto& sample : samples) {

        altitudeLine.append(
            sf::Vertex{
                {
                    screenX(sample.timeSeconds),
                    screenY(sample.altitudeMeters)
                },
                kCyan
            }
        );
    }

    window.draw(altitudeLine);

    // --------------------------------------------------------
    // DRAW AUTOPILOT TARGET
    // --------------------------------------------------------

    // Use separate line segments so there is no false
    // target line while autopilot is disengaged.
    sf::VertexArray targetLines(
        sf::PrimitiveType::Lines
    );

    for (std::size_t i = 1; i < samples.size(); ++i) {

        const auto& previous = samples[i - 1];
        const auto& current = samples[i];

        if (!previous.autopilotEngaged ||
            !current.autopilotEngaged) {
            continue;
        }

        targetLines.append(
            sf::Vertex{
                {
                    screenX(previous.timeSeconds),
                    screenY(previous.targetAltitudeMeters)
                },
                kGreen
            }
        );

        targetLines.append(
            sf::Vertex{
                {
                    screenX(current.timeSeconds),
                    screenY(current.targetAltitudeMeters)
                },
                kGreen
            }
        );
    }

    window.draw(targetLines);

    // --------------------------------------------------------
    // LABEL THE GRAPH
    // --------------------------------------------------------

    drawText(
        window,
        "TOP: " +
            number(maximumAltitude * kMetersToFeet, 0) +
            " ft",
        11,
        {plotLeft, y + 157.0f},
        kMuted
    );

    drawText(
        window,
        "BOTTOM: " +
            number(minimumAltitude * kMetersToFeet, 0) +
            " ft",
        11,
        {plotLeft + 160.0f, y + 157.0f},
        kMuted
    );

    drawText(
        window,
        "ACTUAL: CYAN   TARGET: GREEN",
        11,
        {plotLeft + 355.0f, y + 157.0f},
        kMuted
    );
}

// ============================================================
// DRAW ALL ADVANCED INSTRUMENTS
// ============================================================

void FlightInstruments::draw(
    sf::RenderWindow& window,
    const AircraftState& state,
    const TelemetryRecorder& telemetry
) const {

    drawAttitudeIndicator(window, state);
    drawAltitudeHistory(window, telemetry);
}

} // namespace flight
