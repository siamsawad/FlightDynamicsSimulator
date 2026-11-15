
#pragma once

#include "FlightSim/Aircraft.hpp"
#include "FlightSim/Autopilot.hpp"

#include <SFML/Graphics.hpp>
#include <string>

namespace flight {

// Displays autopilot information on top of the existing
// flight simulator's graphical interface.
class AutopilotOverlay {
public:

    // Load the font needed for the status display.
    AutopilotOverlay();

    // Draw the current autopilot mode and altitude data.
    void draw(
        sf::RenderWindow& window,
        const Autopilot& autopilot,
        const AircraftState& state,
        bool paused
    ) const;

private:

    sf::Font font_;
    bool fontLoaded_ = false;

    // Draw a single line of text.
    void drawText(
        sf::RenderWindow& window,
        const std::string& message,
        unsigned int size,
        sf::Vector2f position,
        sf::Color color
    ) const;
};

} // namespace flight
