#pragma once

#include "FlightSim/SimulationConfig.hpp"

#include <SFML/Graphics.hpp>

#include <array>
#include <cstddef>
#include <string>

namespace flight {

// ============================================================
// SETTINGS SCREEN
// ============================================================
//
// Provides a keyboard-controlled interface for editing
// initial aircraft and environmental parameters.
//
// It does NOT change the active aircraft directly.
//
// Instead, it validates a candidate SimulationConfig
// and returns it to the main application when accepted.
class SettingsScreen {
public:

    SettingsScreen();

    // Open the menu using the current configuration.
    void open(const SimulationConfig& current);

    // Close without applying changes.
    void close();

    // Whether the settings menu is visible.
    bool isOpen() const {
        return open_;
    }

    // Process one SFML event.
    //
    // Returns true ONLY when the user successfully
    // applies a validated configuration.
    //
    // appliedConfig receives the accepted settings.
    bool handleEvent(
        const sf::Event& event,
        SimulationConfig& appliedConfig
    );

    // Draw the complete settings interface.
    void draw(sf::RenderWindow& window) const;

private:

    // Nine editable configuration parameters.
    static constexpr std::size_t kFieldCount = 9;

    bool open_ = false;

    // Currently selected input field.
    std::size_t selectedField_ = 0;

    // Text entered by the user.
    std::array<std::string, kFieldCount> values_{};

    // Message shown after validation failures.
    std::string feedback_;

    // Copy of the configuration being edited.
    SimulationConfig workingConfig_;

    sf::Font font_;
    bool fontLoaded_ = false;

    // Parse, validate, and apply all nine fields.
    bool tryApply(SimulationConfig& appliedConfig);

    // Reusable text drawing function.
    void drawText(
        sf::RenderWindow& window,
        const std::string& message,
        unsigned int size,
        sf::Vector2f position,
        sf::Color color
    ) const;
};

} // namespace flight