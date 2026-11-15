
#include "FlightSim/Autopilot6DOFOverlay.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace flight::sixdof {

namespace {

// Format a number for the HUD.
std::string number(double value, int precision = 0) {

    std::ostringstream stream;

    stream << std::fixed
           << std::setprecision(precision)
           << value;

    return stream.str();
}

// Convert heading radians to 0-360 degrees.
double headingDegrees(double radiansValue) {

    double heading =
        std::fmod(degrees(radiansValue), 360.0);

    if (heading < 0.0) {
        heading += 360.0;
    }

    return heading;
}

} // anonymous namespace

// ============================================================
// CONSTRUCTOR
// ============================================================

Autopilot6DOFOverlay::Autopilot6DOFOverlay() {

    fontLoaded_ = font_.openFromFile(
        "C:/Windows/Fonts/segoeui.ttf"
    );

    if (!fontLoaded_) {

        fontLoaded_ = font_.openFromFile(
            "C:/Windows/Fonts/arial.ttf"
        );
    }
}

// ============================================================
// TEXT HELPER
// ============================================================

void Autopilot6DOFOverlay::drawText(
    sf::RenderWindow& window,
    const std::string& value,
    unsigned int size,
    sf::Vector2f position,
    sf::Color color
) const {

    if (!fontLoaded_) {
        return;
    }

    sf::Text label(font_, value, size);

    label.setPosition(position);
    label.setFillColor(color);

    window.draw(label);
}

// ============================================================
// DRAW AUTOPILOT PANEL
// ============================================================

void Autopilot6DOFOverlay::draw(
    sf::RenderWindow& window,
    const Autopilot6DOF& autopilot,
    const State& state,
    bool paused
) const {

    const auto& status = autopilot.status();

    const EulerAngles attitude =
        toEuler(state.bodyToWorld);

    // Position the panel at the top center.
    constexpr float panelWidth = 610.0f;
    constexpr float panelHeight = 173.0f;

    const float x =
        (
            static_cast<float>(window.getSize().x) -
            panelWidth
        ) * 0.5f;

    constexpr float y = 16.0f;

    // --------------------------------------------------------
    // 1. PANEL BACKGROUND
    // --------------------------------------------------------

    sf::RectangleShape panel({
        panelWidth,
        panelHeight
    });

    panel.setPosition({x, y});

    panel.setFillColor(
        sf::Color(12, 26, 43, 238)
    );

    panel.setOutlineColor(
        status.engaged
            ? sf::Color(86, 209, 155)
            : sf::Color(89, 133, 164)
    );

    panel.setOutlineThickness(1.3f);

    window.draw(panel);

    // Display colors.
    const sf::Color white(239, 246, 255);
    const sf::Color muted(168, 197, 217);
    const sf::Color green(115, 232, 163);
    const sf::Color amber(255, 194, 100);

    // --------------------------------------------------------
    // 2. AUTOPILOT STATUS
    // --------------------------------------------------------

    drawText(
        window,
        "3D AUTOPILOT - ALTITUDE + HEADING HOLD",
        17,
        {x + 15.0f, y + 7.0f},
        white
    );

    drawText(
        window,
        paused
            ? "PAUSED"
            : (status.engaged ? "ENGAGED" : "MANUAL"),
        14,
        {x + 510.0f, y + 8.0f},
        paused
            ? amber
            : (status.engaged ? green : muted)
    );

    // --------------------------------------------------------
    // 3. ALTITUDE
    // --------------------------------------------------------

    const double altitudeFeet =
        -state.positionWorldMeters.z / kFeetToMeters;

    drawText(
        window,
        "TARGET ALT",
        12,
        {x + 15.0f, y + 38.0f},
        muted
    );

    drawText(
        window,
        "CURRENT ALT",
        12,
        {x + 315.0f, y + 38.0f},
        muted
    );

    drawText(
        window,
        status.engaged
            ? number(
                status.targetAltitudeMeters /
                kFeetToMeters
              ) + " ft"
            : "--",
        21,
        {x + 15.0f, y + 53.0f},
        status.engaged ? green : muted
    );

    drawText(
        window,
        number(altitudeFeet) + " ft",
        21,
        {x + 315.0f, y + 53.0f},
        white
    );

    // --------------------------------------------------------
    // 4. HEADING
    // --------------------------------------------------------

    const double currentHeading =
        headingDegrees(attitude.yawRadians);

    drawText(
        window,
        "TARGET HDG",
        12,
        {x + 15.0f, y + 86.0f},
        muted
    );

    drawText(
        window,
        "CURRENT HDG",
        12,
        {x + 315.0f, y + 86.0f},
        muted
    );

    drawText(
        window,
        status.engaged
            ? number(
                headingDegrees(status.targetHeadingRadians)
              ) + " deg"
            : "--",
        18,
        {x + 15.0f, y + 101.0f},
        status.engaged ? green : muted
    );

    drawText(
        window,
        number(currentHeading) + " deg",
        18,
        {x + 315.0f, y + 101.0f},
        white
    );

    // --------------------------------------------------------
    // 5. TRACKING ERRORS AND WARNINGS
    // --------------------------------------------------------

    if (status.engaged) {

        const std::string errorText =
            "ALT ERR " +
            number(
                status.altitudeErrorMeters /
                kFeetToMeters,
                1
            ) +
            " ft  |  HDG ERR " +
            number(
                degrees(status.headingErrorRadians),
                1
            ) + " deg";

        drawText(
            window,
            errorText,
            12,
            {x + 15.0f, y + 128.0f},
            status.lowAirspeed ||
            status.highAngleOfAttack
                ? amber
                : green
        );

        if (status.lowAirspeed ||
            status.highAngleOfAttack) {

            drawText(
                window,
                status.highAngleOfAttack
                    ? "HIGH AOA: RECOVERY REQUEST"
                    : "LOW AIRSPEED: PITCH LIMITED",
                12,
                {x + 326.0f, y + 128.0f},
                amber
            );
        }

    } else {

        drawText(
            window,
            "Manual flight controls active",
            12,
            {x + 15.0f, y + 128.0f},
            muted
        );
    }

    // --------------------------------------------------------
    // 6. KEYBOARD HELP
    // --------------------------------------------------------

    drawText(
        window,
        "H: AP toggle   PgUp/PgDn: 100 ft   J/L: heading 10 deg",
        13,
        {x + 15.0f, y + 150.0f},
        muted
    );
}

} // namespace flight::sixdof
