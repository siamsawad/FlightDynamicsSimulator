
#include "FlightSim/Renderer.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace flight {

namespace {

// Conversion factors for displaying aviation units.
constexpr double kMetersToFeet = 3.280839895;
constexpr double kMetersPerSecondToKnots = 1.943844492;
constexpr double kMetersPerSecondToFeetPerMinute = 196.850394;

// How many screen pixels represent one meter of altitude.
constexpr float kPixelsPerMeter = 0.28f;

// Format a number as a string with chosen decimal places.
std::string formatNumber(double value, int decimals = 1) {
    std::ostringstream stream;

    stream << std::fixed
           << std::setprecision(decimals)
           << value;

    return stream.str();
}

// Draw a rectangular instrument panel.
void drawPanel(
    sf::RenderWindow& window,
    float x,
    float y,
    float width,
    float height
) {
    sf::RectangleShape panel({width, height});

    panel.setPosition({x, y});
    panel.setFillColor(sf::Color(14, 29, 47, 225));
    panel.setOutlineColor(sf::Color(93, 135, 167));
    panel.setOutlineThickness(1.0f);

    window.draw(panel);
}

} // anonymous namespace

// ============================================================
// RENDERER CONSTRUCTOR
// ============================================================

Renderer::Renderer() {

    // Load a font from the Windows font directory.
    // We use Windows fonts because this project currently
    // targets your Windows development environment.
    fontLoaded_ =
        font_.openFromFile("C:/Windows/Fonts/segoeui.ttf");

    // Try Arial if Segoe UI is unavailable.
    if (!fontLoaded_) {
        fontLoaded_ =
            font_.openFromFile("C:/Windows/Fonts/arial.ttf");
    }

    if (!fontLoaded_) {
        std::cerr
            << "Warning: Font not found. HUD text disabled.\n";
    }
}

// ============================================================
// TEXT RENDERING
// ============================================================

void Renderer::drawText(
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
// SCENERY
// ============================================================

void Renderer::drawScenery(
    sf::RenderWindow& window,
    const AircraftState& state
) const {

    const float width =
        static_cast<float>(window.getSize().x);

    const float height =
        static_cast<float>(window.getSize().y);

    // Position where the aircraft appears on the screen.
    const float planeX = width * 0.44f;
    const float planeY = height * 0.43f;

    // Draw several horizontal bands to create a sky gradient.
    // These are visual effects, not part of the physics model.
    for (int i = 0; i < 10; ++i) {

        sf::RectangleShape band({
            width,
            height / 10.0f + 1.0f
        });

        band.setPosition({
            0.0f,
            i * height / 10.0f
        });

        band.setFillColor(sf::Color(
            24 + i * 4,
            64 + i * 5,
            106 + i * 5
        ));

        window.draw(band);
    }

    // Draw a distant sun.
    sf::CircleShape sun(38.0f);

    sun.setPosition({width - 170.0f, 90.0f});
    sun.setFillColor(sf::Color(255, 228, 163, 220));

    window.draw(sun);

    // Draw several cloud groups.
    // Their screen positions shift as the airplane moves.
    // This creates a simple parallax effect.
    const float cloudOffset =
        static_cast<float>(state.positionMeters.x * 0.06);

    for (int i = 0; i < 7; ++i) {

        float x = 90.0f + i * 260.0f - cloudOffset;

        // Wrap clouds around once they move off screen.
        while (x < -150.0f) {
            x += width + 300.0f;
        }

        const float y =
            105.0f + static_cast<float>(i % 3) * 55.0f;

        // Each cloud consists of three flattened circles.
        for (int j = 0; j < 3; ++j) {

            sf::CircleShape cloud(25.0f);

            cloud.setPosition({
                x + j * 28.0f,
                y - (j == 1 ? 13.0f : 0.0f)
            });

            cloud.setScale({1.65f, 0.65f});

            cloud.setFillColor(
                sf::Color(238, 245, 255, 155)
            );

            window.draw(cloud);
        }
    }

    // Calculate where the ground should appear relative
    // to the airplane's real simulated altitude.
    //
    // Higher altitude -> ground appears farther below.
    const float groundY =
        planeY +
        static_cast<float>(state.positionMeters.z) *
        kPixelsPerMeter;

    // Draw the ground only when it enters the screen.
    if (groundY < height) {

        const float visibleGroundY =
            std::max(0.0f, groundY);

        sf::RectangleShape ground({
            width,
            height - visibleGroundY
        });

        ground.setPosition({0.0f, visibleGroundY});
        ground.setFillColor(sf::Color(56, 92, 65));

        window.draw(ground);

        // Ground boundary line.
        if (groundY >= 0.0f) {

            sf::RectangleShape boundary({
                width,
                4.0f
            });

            boundary.setPosition({0.0f, groundY});
            boundary.setFillColor(sf::Color(117, 148, 99));

            window.draw(boundary);
        }

        // Ground markers move relative to aircraft position.
        // These give a visual impression of forward motion.
        for (int i = -5; i < 25; ++i) {

            const double worldX = i * 150.0;

            const float screenX =
                planeX +
                static_cast<float>(
                    worldX - state.positionMeters.x
                ) * kPixelsPerMeter;

            if (screenX < 0.0f || screenX > width) {
                continue;
            }

            sf::RectangleShape marker({18.0f, 3.0f});

            marker.setPosition({
                screenX,
                groundY + 12.0f
            });

            marker.setFillColor(sf::Color(167, 183, 126));

            window.draw(marker);
        }
    }
}

// ============================================================
// AIRCRAFT DRAWING
// ============================================================

void Renderer::drawAircraft(
    sf::RenderWindow& window,
    const AircraftState& state
) const {

    const float width =
        static_cast<float>(window.getSize().x);

    const float height =
        static_cast<float>(window.getSize().y);

    // Keep the airplane near the center of the screen.
    const sf::Vector2f position{
        width * 0.44f,
        height * 0.43f
    };

    // Screen Y increases downward.
    // Therefore positive nose-up pitch must use
    // a negative SFML rotation angle.
    const float rotationDegrees =
        -static_cast<float>(
            degrees(state.pitchRadians)
        );

    // Helper for positioning and rotating aircraft parts.
    auto drawPart = [&](sf::ConvexShape& part) {

        part.setPosition(position);

        part.setRotation(
            sf::degrees(rotationDegrees)
        );

        window.draw(part);
    };

    // -----------------------------
    // Main wing
    // -----------------------------

    sf::ConvexShape wing(4);

    wing.setPoint(0, {-14.0f, 0.0f});
    wing.setPoint(1, { 14.0f, 0.0f});
    wing.setPoint(2, {-24.0f, 37.0f});
    wing.setPoint(3, {-51.0f, 37.0f});

    wing.setFillColor(sf::Color(164, 184, 204));
    wing.setOutlineColor(sf::Color(42, 60, 78));
    wing.setOutlineThickness(1.0f);

    drawPart(wing);

    // -----------------------------
    // Vertical tail
    // -----------------------------

    sf::ConvexShape tail(4);

    tail.setPoint(0, {-66.0f, -2.0f});
    tail.setPoint(1, {-81.0f, -31.0f});
    tail.setPoint(2, {-58.0f, -29.0f});
    tail.setPoint(3, {-38.0f, -2.0f});

    tail.setFillColor(sf::Color(186, 201, 214));
    tail.setOutlineColor(sf::Color(42, 60, 78));
    tail.setOutlineThickness(1.0f);

    drawPart(tail);

    // -----------------------------
    // Aircraft fuselage
    // -----------------------------

    sf::ConvexShape fuselage(8);

    fuselage.setPoint(0, {-77.0f, -5.0f});
    fuselage.setPoint(1, { 43.0f, -5.0f});
    fuselage.setPoint(2, { 68.0f, -2.0f});
    fuselage.setPoint(3, { 84.0f,  2.0f});
    fuselage.setPoint(4, { 68.0f,  7.0f});
    fuselage.setPoint(5, { 43.0f,  9.0f});
    fuselage.setPoint(6, {-72.0f,  8.0f});
    fuselage.setPoint(7, {-82.0f,  2.0f});

    fuselage.setFillColor(sf::Color(227, 234, 241));
    fuselage.setOutlineColor(sf::Color(45, 63, 80));
    fuselage.setOutlineThickness(1.5f);

    drawPart(fuselage);

    // -----------------------------
    // Cockpit window
    // -----------------------------

    sf::ConvexShape cockpit(4);

    cockpit.setPoint(0, {29.0f, -5.0f});
    cockpit.setPoint(1, {48.0f, -4.0f});
    cockpit.setPoint(2, {55.0f,  0.0f});
    cockpit.setPoint(3, {26.0f,  0.0f});

    cockpit.setFillColor(sf::Color(62, 116, 155));

    drawPart(cockpit);
}

// ============================================================
// FLIGHT INSTRUMENTS / HEADS-UP DISPLAY
// ============================================================

void Renderer::drawHUD(
    sf::RenderWindow& window,
    const AircraftState& state,
    const Loads& loads,
    const Controls& controls,
    double simulationTime,
    bool paused
) const {

    const float width =
        static_cast<float>(window.getSize().x);

    const float height =
        static_cast<float>(window.getSize().y);

    const sf::Color white(245, 249, 255);
    const sf::Color muted(170, 196, 215);
    const sf::Color green(111, 230, 170);
    const sf::Color amber(255, 191, 96);

    // Convert physics units into aviation display units.
    const double airspeedKnots =
        loads.airspeedMetersPerSecond *
        kMetersPerSecondToKnots;

    const double altitudeFeet =
        state.positionMeters.z * kMetersToFeet;

    const double verticalSpeed =
        state.velocityMetersPerSecond.z *
        kMetersPerSecondToFeetPerMinute;

    const double pitchDegrees =
        degrees(state.pitchRadians);

    const double alphaDegrees =
        degrees(loads.angleOfAttackRadians);

    // Top title.
    drawText(
        window,
        "FLIGHT DYNAMICS SIMULATOR",
        21,
        {22.0f, 12.0f},
        white
    );

    drawText(
        window,
        "C++  |  2D LONGITUDINAL MODEL",
        14,
        {width - 295.0f, 18.0f},
        muted
    );

    // Airspeed panel.
    drawPanel(window, 20.0f, 65.0f, 235.0f, 128.0f);

    drawText(
        window, "AIRSPEED", 15,
        {34.0f, 77.0f}, muted
    );

    drawText(
        window,
        formatNumber(airspeedKnots, 1) + " knots",
        28,
        {34.0f, 107.0f},
        white
    );

    drawText(
        window, "True airspeed (still air)", 12,
        {34.0f, 163.0f}, muted
    );

    // Altitude panel.
    const float rightX = width - 255.0f;

    drawPanel(window, rightX, 65.0f, 235.0f, 128.0f);

    drawText(
        window, "ALTITUDE", 15,
        {rightX + 14.0f, 77.0f}, muted
    );

    drawText(
        window,
        formatNumber(altitudeFeet, 0) + " ft",
        28,
        {rightX + 14.0f, 107.0f},
        white
    );

    drawText(
        window,
        "V/S " + formatNumber(verticalSpeed, 0) + " ft/min",
        13,
        {rightX + 14.0f, 163.0f},
        muted
    );

    // Throttle panel.
    drawPanel(window, 20.0f, 208.0f, 235.0f, 109.0f);

    drawText(
        window,
        "THROTTLE  " +
            formatNumber(controls.throttle * 100.0, 0) + "%",
        16,
        {34.0f, 220.0f},
        white
    );

    // Throttle background bar.
    sf::RectangleShape throttleBackground({200.0f, 14.0f});
    throttleBackground.setPosition({34.0f, 260.0f});
    throttleBackground.setFillColor(sf::Color(52, 69, 85));

    window.draw(throttleBackground);

    // Throttle fill, proportional to throttle setting.
    const float throttleWidth =
        200.0f * static_cast<float>(
            std::clamp(controls.throttle, 0.0, 1.0)
        );

    sf::RectangleShape throttleFill({
        throttleWidth, 14.0f
    });

    throttleFill.setPosition({34.0f, 260.0f});
    throttleFill.setFillColor(green);

    window.draw(throttleFill);

    // Pitch panel.
    drawPanel(window, rightX, 208.0f, 235.0f, 109.0f);

    drawText(
        window, "PITCH ANGLE", 15,
        {rightX + 14.0f, 220.0f},
        muted
    );

    drawText(
        window,
        formatNumber(pitchDegrees, 2) + " deg",
        27,
        {rightX + 14.0f, 247.0f},
        white
    );

    drawText(
        window,
        "AOA " + formatNumber(alphaDegrees, 1) + " deg",
        13,
        {rightX + 14.0f, 293.0f},
        muted
    );

    // Show a warning when the modeled stall angle
    // has been exceeded.
    if (std::abs(alphaDegrees) > 15.0) {

        drawText(
            window,
            "STALL MODEL ACTIVE",
            18,
            {width * 0.5f - 105.0f, 73.0f},
            sf::Color(255, 109, 109)
        );
    }

    // Bottom information bar.
    sf::RectangleShape bottomBar({width, 72.0f});

    bottomBar.setPosition({0.0f, height - 72.0f});
    bottomBar.setFillColor(sf::Color(13, 26, 43, 242));

    window.draw(bottomBar);

    drawText(
        window,
        "UP/DOWN: Elevator   W/S: Throttle   "
        "P: Pause   R: Reset   ESC: Exit",
        15,
        {22.0f, height - 61.0f},
        white
    );

    drawText(
        window,
        "SIM TIME: " + formatNumber(simulationTime, 1) + " s",
        13,
        {22.0f, height - 30.0f},
        muted
    );

    drawText(
        window,
        "ELEVATOR: " +
            formatNumber(controls.elevator, 2),
        13,
        {245.0f, height - 30.0f},
        muted
    );

    drawText(
        window,
        paused ? "PAUSED" : "SIMULATION RUNNING",
        13,
        {width - 210.0f, height - 30.0f},
        paused ? amber : green
    );
}

// ============================================================
// MAIN RENDERING FUNCTION
// ============================================================

void Renderer::draw(
    sf::RenderWindow& window,
    const AircraftState& state,
    const Loads& loads,
    const Controls& controls,
    double simulationTime,
    bool paused
) const {

    // Clear everything drawn in the previous frame.
    window.clear(sf::Color(25, 63, 101));

    // Draw background first so it remains behind aircraft.
    drawScenery(window, state);

    // Draw the aircraft using its latest physics state.
    drawAircraft(window, state);

    // Draw instruments last so they appear on top.
    drawHUD(
        window,
        state,
        loads,
        controls,
        simulationTime,
        paused
    );
}

} // namespace flight
