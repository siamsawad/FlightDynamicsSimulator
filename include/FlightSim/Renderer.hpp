
#pragma once

#include "FlightSim/Aircraft.hpp"

#include <SFML/Graphics.hpp>
#include <string>

namespace flight {

class Renderer {
public:
    // Constructor: prepares resources such as fonts.
    Renderer();

    // Draw one frame of the flight simulator.
    // All inputs are read-only.
    void draw(
        sf::RenderWindow& window,
        const AircraftState& state,
        const Loads& loads,
        const Controls& controls,
        double simulationTime,
        bool paused
    ) const;

private:
    // Font used for flight instruments.
    sf::Font font_;
    bool fontLoaded_ = false;

    // Draw the sky, clouds and ground.
    void drawScenery(
        sf::RenderWindow& window,
        const AircraftState& state
    ) const;

    // Draw the aircraft using its actual pitch angle.
    void drawAircraft(
        sf::RenderWindow& window,
        const AircraftState& state
    ) const;

    // Draw the flight instruments and control information.
    void drawHUD(
        sf::RenderWindow& window,
        const AircraftState& state,
        const Loads& loads,
        const Controls& controls,
        double simulationTime,
        bool paused
    ) const;

    // Helper function for drawing text on screen.
    void drawText(
        sf::RenderWindow& window,
        const std::string& message,
        unsigned int size,
        sf::Vector2f position,
        sf::Color color
    ) const;
};

} // namespace flight
