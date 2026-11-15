
#include "FlightSim/SettingsScreen.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace flight {

namespace {

// ============================================================
// FIELD DESCRIPTIONS
// ============================================================

// The order must match the values array in open()
// and the parsing order in tryApply().
constexpr const char* kLabels[9] = {
    "Initial altitude (ft)",
    "Initial airspeed (knots)",
    "Autopilot target (ft)",
    "Aircraft mass (kg)",
    "Wingspan (m)",
    "Wing area (m^2)",
    "Maximum thrust (N)",
    "Headwind + / tailwind - (kt)",
    "Air density multiplier"
};

// Convert a number into editable text.
//
// Use enough precision to preserve our original starting
// values without showing unnecessary trailing zeroes.
std::string editableNumber(double value) {

    std::ostringstream stream;

    stream << std::fixed
           << std::setprecision(7)
           << value;

    std::string result = stream.str();

    // Remove unnecessary trailing zeroes.
    while (result.find('.') != std::string::npos &&
           !result.empty() &&
           result.back() == '0') {

        result.pop_back();
    }

    // Remove a trailing decimal point.
    if (!result.empty() && result.back() == '.') {
        result.pop_back();
    }

    return result;
}

// Parse a complete floating-point number.
//
// std::stod alone may accept a valid prefix followed
// by invalid characters. We require the WHOLE string
// to be a valid finite number.
bool parseNumber(
    const std::string& text,
    double& result
) {

    if (text.empty()) {
        return false;
    }

    try {

        std::size_t parsedCharacters = 0;

        const double value = std::stod(
            text,
            &parsedCharacters
        );

        if (parsedCharacters != text.size() ||
            !std::isfinite(value)) {
            return false;
        }

        result = value;
        return true;

    } catch (const std::exception&) {

        return false;
    }
}

} // anonymous namespace

// ============================================================
// CONSTRUCTOR
// ============================================================

SettingsScreen::SettingsScreen() {

    // Use our established Windows font approach.
    fontLoaded_ =
        font_.openFromFile("C:/Windows/Fonts/segoeui.ttf");

    if (!fontLoaded_) {
        fontLoaded_ =
            font_.openFromFile("C:/Windows/Fonts/arial.ttf");
    }
}

// ============================================================
// OPEN SETTINGS
// ============================================================

void SettingsScreen::open(
    const SimulationConfig& current
) {

    // Work on a copy, not on the active simulation.
    workingConfig_ = current;

    selectedField_ = 0;
    feedback_.clear();

    // Fill the editing fields with current values.
    values_[0] = editableNumber(
        current.initialAltitudeFeet
    );

    values_[1] = editableNumber(
        current.initialAirspeedKnots
    );

    values_[2] = editableNumber(
        current.autopilotTargetFeet
    );

    values_[3] = editableNumber(
        current.aircraft.massKg
    );

    values_[4] = editableNumber(
        current.aircraft.wingSpanMeters
    );

    values_[5] = editableNumber(
        current.aircraft.wingAreaM2
    );

    values_[6] = editableNumber(
        current.aircraft.maxThrustNewtons
    );

    values_[7] = editableNumber(
        current.environment.headwindKnots
    );

    values_[8] = editableNumber(
        current.environment.airDensityMultiplier
    );

    open_ = true;
}

// ============================================================
// CLOSE SETTINGS
// ============================================================

void SettingsScreen::close() {

    // Closing without applying discards edited values.
    // The active configuration remains unchanged.
    open_ = false;
    feedback_.clear();
}

// ============================================================
// VALIDATE AND APPLY
// ============================================================

bool SettingsScreen::tryApply(
    SimulationConfig& appliedConfig
) {

    // Create another temporary copy.
    //
    // If a value is invalid, neither the active
    // configuration nor workingConfig_ is modified.
    SimulationConfig candidate = workingConfig_;

    double parsed[9]{};

    // Parse every field before changing the candidate.
    for (std::size_t i = 0; i < kFieldCount; ++i) {

        if (!parseNumber(values_[i], parsed[i])) {

            feedback_ =
                std::string("Invalid number: ") +
                kLabels[i];

            selectedField_ = i;
            return false;
        }
    }

    // --------------------------------------------------------
    // TRANSFER VALUES TO THE CANDIDATE CONFIGURATION
    // --------------------------------------------------------

    candidate.initialAltitudeFeet = parsed[0];
    candidate.initialAirspeedKnots = parsed[1];
    candidate.autopilotTargetFeet = parsed[2];

    candidate.aircraft.massKg = parsed[3];
    candidate.aircraft.wingSpanMeters = parsed[4];
    candidate.aircraft.wingAreaM2 = parsed[5];
    candidate.aircraft.maxThrustNewtons = parsed[6];

    candidate.environment.headwindKnots = parsed[7];
    candidate.environment.airDensityMultiplier = parsed[8];

    // Validate ranges and aircraft geometry.
    const ConfigValidation validation =
        validateSimulationConfig(candidate);

    if (!validation.valid) {

        feedback_ = validation.message;
        return false;
    }

    // All values are acceptable.
    appliedConfig = candidate;

    // Close the settings screen after successful apply.
    open_ = false;
    feedback_.clear();

    return true;
}

// ============================================================
// PROCESS KEYBOARD AND TEXT EVENTS
// ============================================================

bool SettingsScreen::handleEvent(
    const sf::Event& event,
    SimulationConfig& appliedConfig
) {

    if (!open_) {
        return false;
    }

    // --------------------------------------------------------
    // 1. HANDLE SPECIAL KEYS
    // --------------------------------------------------------

    if (const auto* key =
            event.getIf<sf::Event::KeyPressed>()) {

        const auto code = key->code;

        // ESC: Cancel without changing the simulation.
        if (code == sf::Keyboard::Key::Escape) {

            close();
            return false;
        }

        // TAB or DOWN: Move to the next field.
        if (code == sf::Keyboard::Key::Tab ||
            code == sf::Keyboard::Key::Down) {

            selectedField_ =
                (selectedField_ + 1) % kFieldCount;

            feedback_.clear();
            return false;
        }

        // UP: Move to the previous field.
        if (code == sf::Keyboard::Key::Up) {

            selectedField_ =
                (selectedField_ + kFieldCount - 1)
                % kFieldCount;

            feedback_.clear();
            return false;
        }

        // BACKSPACE: Remove the last typed character.
        if (code == sf::Keyboard::Key::Backspace) {

            if (!values_[selectedField_].empty()) {
                values_[selectedField_].pop_back();
            }

            feedback_.clear();
            return false;
        }

        // DELETE: Clear the entire selected field.
        if (code == sf::Keyboard::Key::Delete) {

            values_[selectedField_].clear();

            feedback_.clear();
            return false;
        }

        // ENTER: Validate and apply settings.
        if (code == sf::Keyboard::Key::Enter) {

            return tryApply(appliedConfig);
        }
    }

    // --------------------------------------------------------
    // 2. HANDLE TYPED CHARACTERS
    // --------------------------------------------------------

    if (const auto* typed =
            event.getIf<sf::Event::TextEntered>()) {

        const char32_t unicode = typed->unicode;

        // Accept only ASCII digits, decimal points,
        // and signs for our numerical input fields.
        //
        // Backspace is handled by KeyPressed above,
        // so non-printable text events are ignored.
        const bool validCharacter =
            (unicode >= U'0' && unicode <= U'9') ||
            unicode == U'.' ||
            unicode == U'-' ||
            unicode == U'+';

        if (!validCharacter) {
            return false;
        }

        // Prevent excessively long input strings.
        if (values_[selectedField_].size() >= 18) {
            return false;
        }

        values_[selectedField_].push_back(
            static_cast<char>(unicode)
        );

        feedback_.clear();
    }

    return false;
}

// ============================================================
// DRAW TEXT
// ============================================================

void SettingsScreen::drawText(
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
// DRAW SETTINGS SCREEN
// ============================================================

void SettingsScreen::draw(
    sf::RenderWindow& window
) const {

    if (!open_) {
        return;
    }

    const float width =
        static_cast<float>(window.getSize().x);

    const float height =
        static_cast<float>(window.getSize().y);

    // Dark overlay covers the existing flight display.
    sf::RectangleShape overlay({width, height});

    overlay.setFillColor(sf::Color(5, 12, 23, 238));

    window.draw(overlay);

    // Main settings panel.
    constexpr float panelWidth = 740.0f;
    constexpr float panelHeight = 628.0f;

    const float x =
        (width - panelWidth) / 2.0f;

    const float y = 46.0f;

    sf::RectangleShape panel({
        panelWidth,
        panelHeight
    });

    panel.setPosition({x, y});
    panel.setFillColor(sf::Color(18, 34, 53));

    panel.setOutlineColor(sf::Color(89, 149, 188));
    panel.setOutlineThickness(1.5f);

    window.draw(panel);

    // Standard interface colors.
    const sf::Color white(240, 247, 255);
    const sf::Color muted(163, 192, 214);
    const sf::Color green(111, 230, 170);
    const sf::Color warning(255, 155, 117);

    // --------------------------------------------------------
    // TITLE AND DESCRIPTION
    // --------------------------------------------------------

    drawText(
        window,
        "FLIGHT CONFIGURATION",
        24,
        {x + 23.0f, y + 12.0f},
        white
    );

    drawText(
        window,
        "Edit values, then press ENTER to validate and start.",
        15,
        {x + 23.0f, y + 53.0f},
        muted
    );

    // --------------------------------------------------------
    // NINE EDITABLE FIELDS
    // --------------------------------------------------------

    for (std::size_t i = 0; i < kFieldCount; ++i) {

        const float rowY =
            y + 94.0f + static_cast<float>(i) * 45.0f;

        const bool selected = (i == selectedField_);

        // Highlight the currently selected row.
        if (selected) {

            sf::RectangleShape highlight({
                panelWidth - 36.0f,
                37.0f
            });

            highlight.setPosition({
                x + 18.0f,
                rowY - 3.0f
            });

            highlight.setFillColor(
                sf::Color(38, 76, 103)
            );

            window.draw(highlight);
        }

        // Field label.
        drawText(
            window,
            kLabels[i],
            17,
            {x + 27.0f, rowY + 2.0f},
            selected ? white : muted
        );

        // Text box containing the current value.
        sf::RectangleShape inputBox({
            215.0f,
            31.0f
        });

        inputBox.setPosition({
            x + 491.0f,
            rowY
        });

        inputBox.setFillColor(
            selected
                ? sf::Color(29, 59, 80)
                : sf::Color(12, 25, 40)
        );

        inputBox.setOutlineColor(
            selected
                ? green
                : sf::Color(72, 101, 128)
        );

        inputBox.setOutlineThickness(1.0f);

        window.draw(inputBox);

        // Render the entered numerical value.
        drawText(
            window,
            values_[i],
            18,
            {x + 502.0f, rowY + 2.0f},
            white
        );
    }

    // --------------------------------------------------------
    // VALIDATION FEEDBACK
    // --------------------------------------------------------

    if (!feedback_.empty()) {

        drawText(
            window,
            feedback_,
            14,
            {x + 23.0f, y + 523.0f},
            warning
        );

    } else {

        drawText(
            window,
            "Only supported ranges are accepted.",
            14,
            {x + 23.0f, y + 523.0f},
            muted
        );
    }

    // --------------------------------------------------------
    // KEYBOARD HELP
    // --------------------------------------------------------

    drawText(
        window,
        "TAB / UP / DOWN: Select    BACKSPACE: Edit    DEL: Clear",
        14,
        {x + 23.0f, y + 553.0f},
        muted
    );

    drawText(
        window,
        "ENTER: Apply & Restart       ESC: Cancel",
        15,
        {x + 23.0f, y + 583.0f},
        green
    );
}

} // namespace flight
