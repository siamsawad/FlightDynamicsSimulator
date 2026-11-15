
#include "FlightSim/AutopilotOverlay.hpp"
#include "FlightSim/Telemetry.hpp"

#include <iomanip>
#include <iostream>
#include <sstream>

namespace flight {

namespace {

std::string formatValue(double value, int decimals = 0) {

    std::ostringstream stream;

    stream << std::fixed
           << std::setprecision(decimals)
           << value;

    return stream.str();
}

} // anonymous namespace

// ============================================================
// CONSTRUCTOR
// ============================================================

AutopilotOverlay::AutopilotOverlay() {

    fontLoaded_ =
        font_.openFromFile("C:/Windows/Fonts/segoeui.ttf");

    if (!fontLoaded_) {
        fontLoaded_ =
            font_.openFromFile("C:/Windows/Fonts/arial.ttf");
    }

    if (!fontLoaded_) {
        std::cerr
            << "Warning: Autopilot HUD font not found.\n";
    }
}

// ============================================================
// TEXT HELPER
// ============================================================

void AutopilotOverlay::drawText(
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
// AUTOPILOT PANEL
// ============================================================

void AutopilotOverlay::draw(
    sf::RenderWindow& window,
    const Autopilot& autopilot,
    const AircraftState& state,
    bool paused
) const {

    constexpr float panelWidth = 440.0f;
    constexpr float panelHeight = 126.0f;

    const float windowWidth =
        static_cast<float>(window.getSize().x);

    const float x =
        (windowWidth - panelWidth) / 2.0f;

    const float y = 63.0f;

    const bool engaged = autopilot.isEngaged();

    sf::RectangleShape panel({
        panelWidth,
        panelHeight
    });

    panel.setPosition({x, y});
    panel.setFillColor(sf::Color(14, 29, 47, 235));

    panel.setOutlineColor(
        engaged
            ? sf::Color(91, 209, 159)
            : sf::Color(93, 135, 167)
    );

    panel.setOutlineThickness(1.5f);

    window.draw(panel);

    const sf::Color white(245, 249, 255);
    const sf::Color muted(170, 196, 215);
    const sf::Color green(111, 230, 170);
    const sf::Color amber(255, 191, 96);

    // Mode status.
    drawText(
        window,
        "AUTOPILOT / ALTITUDE HOLD",
        16,
        {x + 15.0f, y + 9.0f},
        white
    );

    drawText(
        window,
        paused ? "PAUSED" :
            (engaged ? "ENGAGED" : "MANUAL"),
        14,
        {x + 335.0f, y + 10.0f},
        paused ? amber : (engaged ? green : muted)
    );

    // All visible altitude values use FEET.
    drawText(
        window,
        "TARGET ALTITUDE (FT)",
        12,
        {x + 15.0f, y + 41.0f},
        muted
    );

    drawText(
        window,
        "CURRENT ALTITUDE (FT)",
        12,
        {x + 230.0f, y + 41.0f},
        muted
    );

    // Convert target from meters to feet for display.
    std::string targetText = "--";

    if (engaged) {

        const double targetFeet =
            autopilot.targetAltitudeMeters() *
            kMetersToFeet;

        targetText = formatValue(targetFeet) + " ft";
    }

    const double currentAltitudeFeet =
        state.positionMeters.z * kMetersToFeet;

    drawText(
        window,
        targetText,
        23,
        {x + 15.0f, y + 57.0f},
        engaged ? green : muted
    );

    drawText(
        window,
        formatValue(currentAltitudeFeet) + " ft",
        23,
        {x + 230.0f, y + 57.0f},
        white
    );

    if (engaged) {

        const double errorFeet =
            autopilot.altitudeErrorMeters(state) *
            kMetersToFeet;

        const std::string sign =
            errorFeet >= 0.0 ? "+" : "";

        drawText(
            window,
            "ERROR: " + sign +
                formatValue(errorFeet, 1) + " ft",
            12,
            {x + 15.0f, y + 91.0f},
            green
        );

    } else {

        drawText(
            window,
            "Manual elevator control",
            12,
            {x + 15.0f, y + 91.0f},
            muted
        );
    }

    drawText(
        window,
        "A: Toggle | PgUp/PgDn: 100 ft | Arrows: Manual",
        11,
        {x + 15.0f, y + 108.0f},
        muted
    );
}

} // namespace flight
