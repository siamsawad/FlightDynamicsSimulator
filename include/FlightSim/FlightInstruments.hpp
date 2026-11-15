
#pragma once

#include "FlightSim/Aircraft.hpp"
#include "FlightSim/Telemetry.hpp"

#include <SFML/Graphics.hpp>
#include <string>

namespace flight {

class FlightInstruments {
public:

    FlightInstruments();

    // Draw advanced instruments over the existing scene.
    void draw(
        sf::RenderWindow& window,
        const AircraftState& state,
        const TelemetryRecorder& telemetry
    ) const;

private:

    sf::Font font_;
    bool fontLoaded_ = false;

    // Draw the pitch-only artificial horizon.
    void drawAttitudeIndicator(
        sf::RenderWindow& window,
        const AircraftState& state
    ) const;

    // Draw altitude and autopilot target history.
    void drawAltitudeHistory(
        sf::RenderWindow& window,
        const TelemetryRecorder& telemetry
    ) const;

    // Reusable text helper.
    void drawText(
        sf::RenderWindow& window,
        const std::string& message,
        unsigned int size,
        sf::Vector2f position,
        sf::Color color
    ) const;
};

} // namespace flight
