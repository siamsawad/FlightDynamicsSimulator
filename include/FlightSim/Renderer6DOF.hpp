
#pragma once

#include "FlightSim/Dynamics6DOF.hpp"
#include "FlightSim/Projection3D.hpp"
#include "FlightSim/Visuals13.hpp"

#include <SFML/Graphics.hpp>

#include <deque>
#include <string>

namespace flight::sixdof {

class Renderer6DOF {
public:

    Renderer6DOF();

    // Draw the complete 3D flight simulation.
    void draw(
        sf::RenderWindow& window,
        const State& state,
        const ForcesMoments& loads,
        const Inputs& controls,
        const std::deque<Vec3>& flightTrail,
        double simulationSeconds,
        bool paused
    ) const;

private:

    sf::Font font_;
    bool fontLoaded_ = false;

    // Version 13: PFD and environmental visuals.
    Visuals13 visuals13_;

    // Draw sky, ground, scenery, and flight trail.
    void drawScene(
        sf::RenderWindow& window,
        const State& state,
        const Camera3D& camera,
        const std::deque<Vec3>& trail
    ) const;

    // Draw the aircraft using projected 3D triangles.
    void drawAircraft(
        sf::RenderWindow& window,
        const State& state,
        const Camera3D& camera
    ) const;

    // Draw existing flight instruments and keyboard help.
    void drawHUD(
        sf::RenderWindow& window,
        const State& state,
        const ForcesMoments& loads,
        const Inputs& controls,
        double timeSeconds,
        bool paused
    ) const;

    // Reusable text helper.
    void text(
        sf::RenderWindow& window,
        const std::string& value,
        unsigned int size,
        sf::Vector2f position,
        sf::Color color
    ) const;
};

} // namespace flight::sixdof
