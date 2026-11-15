
#pragma once

#include "FlightSim/Autopilot6DOF.hpp"

#include <SFML/Graphics.hpp>
#include <string>

namespace flight::sixdof {

// ============================================================
// 3D AUTOPILOT STATUS DISPLAY
// ============================================================

class Autopilot6DOFOverlay {
public:

    Autopilot6DOFOverlay();

    // Draw target altitude, heading, tracking errors,
    // autopilot mode, and relevant warnings.
    void draw(
        sf::RenderWindow& window,
        const Autopilot6DOF& autopilot,
        const State& state,
        bool paused
    ) const;

private:

    sf::Font font_;
    bool fontLoaded_ = false;

    // Reusable text drawing helper.
    void drawText(
        sf::RenderWindow& window,
        const std::string& value,
        unsigned int size,
        sf::Vector2f position,
        sf::Color color
    ) const;
};

} // namespace flight::sixdof
