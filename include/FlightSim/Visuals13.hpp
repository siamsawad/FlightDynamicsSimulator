
#pragma once

#include "FlightSim/Dynamics6DOF.hpp"
#include "FlightSim/Projection3D.hpp"

#include <SFML/Graphics.hpp>
#include <string>

namespace flight::sixdof {

// Draws presentation-only graphics.
// Never changes the aircraft's physical state.
class Visuals13 {
public:
    Visuals13();

    // Draw sky, clouds, and distant terrain.
    void drawEnvironment(
        sf::RenderWindow& window,
        const State& state,
        const Camera3D& camera,
        float horizonY
    ) const;

    // Draw the primary flight display using actual
    // six-degree-of-freedom flight measurements.
    void drawPFD(
        sf::RenderWindow& window,
        const State& state,
        const ForcesMoments& loads
    ) const;

private:
    sf::Font font_;
    bool fontLoaded_ = false;

    void drawText(
        sf::RenderWindow& window,
        const std::string& value,
        unsigned int size,
        sf::Vector2f position,
        sf::Color color
    ) const;
};

} // namespace flight::sixdof
