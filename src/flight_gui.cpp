
#include "FlightSim/Aircraft.hpp"
#include "FlightSim/Aerodynamics.hpp"
#include "FlightSim/Autopilot.hpp"
#include "FlightSim/AutopilotOverlay.hpp"
#include "FlightSim/ControlSystem.hpp"
#include "FlightSim/FlightInstruments.hpp"
#include "FlightSim/PhysicsEngine.hpp"
#include "FlightSim/Renderer.hpp"
#include "FlightSim/SettingsScreen.hpp"
#include "FlightSim/SimulationConfig.hpp"
#include "FlightSim/Telemetry.hpp"

#include <SFML/Graphics.hpp>

#include <algorithm>
#include <iostream>

int main() {

    using namespace flight;

    // ========================================================
    // 1. CREATE GRAPHICAL WINDOW
    // ========================================================

    sf::RenderWindow window(
        sf::VideoMode({1200u, 720u}),
        "Flight Dynamics Simulator - Version 8",
        sf::Style::Titlebar | sf::Style::Close
    );

    window.setFramerateLimit(60);

    // ========================================================
    // 2. SIMULATION CONFIGURATION
    // ========================================================

    // NEW: Stores the selected aircraft and environment.
    SimulationConfig config;

    // Create the initial aircraft state from user settings.
    AircraftState state =
        makeInitialAircraftState(config);

    // The aircraft parameters now come from config.
    // They are no longer separately hardcoded in main().
    Controls controls;

    const ControlSystem controlSystem;
    Autopilot autopilot;

    // ========================================================
    // 3. GRAPHICAL COMPONENTS
    // ========================================================

    Renderer renderer;
    AutopilotOverlay autopilotOverlay;
    FlightInstruments flightInstruments;

    // NEW: Interactive configuration interface.
    SettingsScreen settingsScreen;

    // ========================================================
    // 4. FLIGHT TELEMETRY
    // ========================================================

    TelemetryRecorder telemetry(
        "flight_gui_telemetry.csv"
    );

    // ========================================================
    // 5. SIMULATION TIMING
    // ========================================================

    sf::Clock frameClock;

    double accumulator = 0.0;
    double simulationSeconds = 0.0;

    int physicsSteps = 0;

    bool paused = false;

    // Remember whether simulation was paused before
    // the settings menu opened.
    bool pausedBeforeSettings = false;

    // Prevent ESC used to cancel the menu from also
    // closing the simulator.
    bool blockEscapeUntilReleased = false;

    // Physics frequency: 120 Hz.
    // Telemetry frequency: 10 Hz.
    constexpr int kStepsPerSample = 12;

    // Autopilot target adjustment: 100 feet.
    constexpr double kTargetStepMeters =
        100.0 * kFeetToMeters;

    // Previous keyboard states for one-press actions.
    bool previousPauseHeld = false;
    bool previousResetHeld = false;
    bool previousAutopilotHeld = false;
    bool previousPageUpHeld = false;
    bool previousPageDownHeld = false;

    // ========================================================
    // 6. HELPER: RECORD THE CURRENT STARTING STATE
    // ========================================================

    // This lambda lets us reuse the same recording code
    // after a reset or configuration change.
    auto recordInitialState = [&]() {

        const Loads loads = calculateLoads(
            state,
            config.aircraft,
            controls,
            config.environment
        );

        telemetry.recordSample(
            simulationSeconds,
            state,
            loads,
            controls,
            autopilot
        );
    };

    // ========================================================
    // 7. HELPER: START A NEW FLIGHT
    // ========================================================

    auto resetSimulation = [&]() {

        // Rebuild the aircraft using CURRENT configuration.
        state = makeInitialAircraftState(config);

        // Restore throttle and elevator defaults.
        controls = Controls{};

        // Disengage and reset autopilot state.
        autopilot = Autopilot{};

        // Reset simulated time.
        accumulator = 0.0;
        simulationSeconds = 0.0;
        physicsSteps = 0;

        paused = false;

        // Keep earlier CSV records under previous run IDs.
        telemetry.startNewRun();

        // Log the new aircraft starting conditions.
        recordInitialState();

        std::cout
            << "Simulation restarted.\n"
            << "Starting altitude: "
            << config.initialAltitudeFeet << " ft\n"
            << "Starting airspeed: "
            << config.initialAirspeedKnots << " knots\n"
            << "Headwind: "
            << config.environment.headwindKnots << " knots\n"
            << "Density multiplier: "
            << config.environment.airDensityMultiplier
            << "\n";
    };

    // Log the initial aircraft state.
    recordInitialState();

    std::cout
        << "Flight Dynamics Simulator - Version 8\n"
        << "F2: Open flight configuration\n"
        << "A: Toggle altitude-hold autopilot\n"
        << "PageUp/PageDown: Target +/-100 ft\n"
        << "Up/Down: Manual elevator\n"
        << "W/S: Throttle\n"
        << "P: Pause | R: Reset | ESC: Exit\n";

    // ========================================================
    // 8. MAIN APPLICATION LOOP
    // ========================================================

    while (window.isOpen()) {

        // Tracks whether the settings menu was opened,
        // closed, or applied during the current frame.
        bool settingsTouchedThisFrame = false;

        // ----------------------------------------------------
        // A. PROCESS SFML EVENTS
        // ----------------------------------------------------

        while (const auto event = window.pollEvent()) {

            // Window close button always works.
            if (event->is<sf::Event::Closed>()) {

                window.close();
                break;
            }

            // ------------------------------------------------
            // SETTINGS MENU EVENTS
            // ------------------------------------------------

            if (settingsScreen.isOpen()) {

                settingsTouchedThisFrame = true;

                // The menu edits a separate candidate.
                SimulationConfig acceptedConfig = config;

                const bool applied =
                    settingsScreen.handleEvent(
                        *event,
                        acceptedConfig
                    );

                if (applied) {

                    // Validation succeeded.
                    // Replace active configuration.
                    config = acceptedConfig;

                    // Start a NEW flight with the selected
                    // aircraft and environmental conditions.
                    resetSimulation();

                    std::cout
                        << "New configuration applied.\n";

                } else if (!settingsScreen.isOpen()) {

                    // User pressed ESC to cancel.
                    // Restore the previous pause state.
                    paused = pausedBeforeSettings;

                    accumulator = 0.0;

                    blockEscapeUntilReleased = true;

                    std::cout
                        << "Settings cancelled.\n";
                }

                // Settings events must not also control
                // the aircraft in the same frame.
                continue;
            }

            // ------------------------------------------------
            // OPEN SETTINGS WITH F2
            // ------------------------------------------------

            if (const auto* key =
                    event->getIf<sf::Event::KeyPressed>()) {

                if (key->code == sf::Keyboard::Key::F2) {

                    pausedBeforeSettings = paused;

                    // Pause physics while editing.
                    paused = true;
                    accumulator = 0.0;

                    settingsScreen.open(config);

                    settingsTouchedThisFrame = true;
                }
            }
        }

        if (!window.isOpen()) {
            break;
        }

        // ----------------------------------------------------
        // B. MEASURE FRAME TIME
        // ----------------------------------------------------

        const double frameSeconds = std::min(
            0.1,
            static_cast<double>(
                frameClock.restart().asSeconds()
            )
        );

        // Release the special ESC guard once ESC is up.
        if (!sf::Keyboard::isKeyPressed(
                sf::Keyboard::Key::Escape)) {

            blockEscapeUntilReleased = false;
        }

        // ----------------------------------------------------
        // C. READ MANUAL INPUTS
        // ----------------------------------------------------

        PilotInput pilotInput;

        // Do not process flight controls during menu
        // interaction, even if the menu just closed.
        const bool flightInputEnabled =
            !settingsScreen.isOpen() &&
            !settingsTouchedThisFrame;

        if (flightInputEnabled) {

            if (!window.hasFocus()) {

                paused = true;

                previousPauseHeld = false;
                previousResetHeld = false;
                previousAutopilotHeld = false;
                previousPageUpHeld = false;
                previousPageDownHeld = false;

            } else {

                // --------------------------------------------
                // ESC: CLOSE SIMULATOR
                // --------------------------------------------

                if (!blockEscapeUntilReleased &&
                    sf::Keyboard::isKeyPressed(
                        sf::Keyboard::Key::Escape)) {

                    window.close();
                    break;
                }

                // --------------------------------------------
                // P: PAUSE / RESUME
                // --------------------------------------------

                const bool pauseHeld =
                    sf::Keyboard::isKeyPressed(
                        sf::Keyboard::Key::P
                    );

                if (pauseHeld && !previousPauseHeld) {
                    paused = !paused;
                }

                previousPauseHeld = pauseHeld;

                // --------------------------------------------
                // R: RESET CURRENT CONFIGURATION
                // --------------------------------------------

                const bool resetHeld =
                    sf::Keyboard::isKeyPressed(
                        sf::Keyboard::Key::R
                    );

                if (resetHeld && !previousResetHeld) {

                    // Keep user-selected settings.
                    resetSimulation();
                }

                previousResetHeld = resetHeld;

                // --------------------------------------------
                // A: TOGGLE AUTOPILOT
                // --------------------------------------------

                const bool autopilotHeld =
                    sf::Keyboard::isKeyPressed(
                        sf::Keyboard::Key::A
                    );

                if (!paused &&
                    autopilotHeld &&
                    !previousAutopilotHeld) {

                    if (autopilot.isEngaged()) {

                        autopilot.disengage();

                        std::cout
                            << "Autopilot disengaged.\n";

                    } else {

                        // Engage using the current state.
                        autopilot.engage(state);

                        // NEW:
                        // Replace the captured altitude
                        // with the configured AP target.
                        autopilot.setTargetAltitudeMeters(
                            config.autopilotTargetFeet *
                            kFeetToMeters
                        );

                        std::cout
                            << "Autopilot engaged.\n";
                    }
                }

                previousAutopilotHeld = autopilotHeld;

                // --------------------------------------------
                // PAGE UP: RAISE TARGET BY 100 FEET
                // --------------------------------------------

                const bool pageUpHeld =
                    sf::Keyboard::isKeyPressed(
                        sf::Keyboard::Key::PageUp
                    );

                if (!paused &&
                    autopilot.isEngaged() &&
                    pageUpHeld &&
                    !previousPageUpHeld) {

                    autopilot.adjustTargetAltitudeMeters(
                        kTargetStepMeters
                    );
                }

                previousPageUpHeld = pageUpHeld;

                // --------------------------------------------
                // PAGE DOWN: LOWER TARGET BY 100 FEET
                // --------------------------------------------

                const bool pageDownHeld =
                    sf::Keyboard::isKeyPressed(
                        sf::Keyboard::Key::PageDown
                    );

                if (!paused &&
                    autopilot.isEngaged() &&
                    pageDownHeld &&
                    !previousPageDownHeld) {

                    autopilot.adjustTargetAltitudeMeters(
                        -kTargetStepMeters
                    );
                }

                previousPageDownHeld = pageDownHeld;

                // --------------------------------------------
                // CONTINUOUS MANUAL FLIGHT CONTROLS
                // --------------------------------------------

                if (!paused) {

                    const bool pitchUp =
                        sf::Keyboard::isKeyPressed(
                            sf::Keyboard::Key::Up
                        );

                    const bool pitchDown =
                        sf::Keyboard::isKeyPressed(
                            sf::Keyboard::Key::Down
                        );

                    // Manual input disengages autopilot.
                    if ((pitchUp || pitchDown) &&
                        autopilot.isEngaged()) {

                        autopilot.disengage();

                        std::cout
                            << "Manual elevator override.\n";
                    }

                    if (pitchUp) {
                        pilotInput.elevatorAxis += 1.0;
                    }

                    if (pitchDown) {
                        pilotInput.elevatorAxis -= 1.0;
                    }

                    // W: Increase throttle.
                    if (sf::Keyboard::isKeyPressed(
                            sf::Keyboard::Key::W)) {

                        pilotInput.throttleAxis += 1.0;
                    }

                    // S: Decrease throttle.
                    if (sf::Keyboard::isKeyPressed(
                            sf::Keyboard::Key::S)) {

                        pilotInput.throttleAxis -= 1.0;
                    }
                }
            }
        }

        // ----------------------------------------------------
        // D. FIXED-TIMESTEP FLIGHT PHYSICS
        // ----------------------------------------------------

        if (!paused && flightInputEnabled) {

            accumulator += frameSeconds;

            while (accumulator >= kTimeStep) {

                PilotInput stepInput = pilotInput;

                // Autopilot supplies elevator request.
                if (autopilot.isEngaged()) {

                    stepInput.elevatorAxis =
                        autopilot.update(
                            state,
                            kTimeStep
                        );
                }

                // Existing rate-limited control system.
                controlSystem.update(
                    controls,
                    stepInput,
                    kTimeStep
                );

                // NEW: Apply the configured aircraft
                // and environmental parameters.
                const Loads loads =
                    calculateLoads(
                        state,
                        config.aircraft,
                        controls,
                        config.environment
                    );

                // Measure autopilot error each physics step.
                telemetry.updateTracking(
                    state,
                    autopilot,
                    kTimeStep
                );

                // Existing physics integration.
                stepPhysics(
                    state,
                    config.aircraft,
                    loads,
                    kTimeStep
                );

                simulationSeconds += kTimeStep;
                accumulator -= kTimeStep;
                ++physicsSteps;

                // Log 10 telemetry samples per second.
                if (physicsSteps % kStepsPerSample == 0) {

                    const Loads measuredLoads =
                        calculateLoads(
                            state,
                            config.aircraft,
                            controls,
                            config.environment
                        );

                    telemetry.recordSample(
                        simulationSeconds,
                        state,
                        measuredLoads,
                        controls,
                        autopilot
                    );
                }
            }

        } else {

            // No physics time accumulates while paused
            // or interacting with the settings menu.
            accumulator = 0.0;
        }

        // ----------------------------------------------------
        // E. DRAW SIMULATOR
        // ----------------------------------------------------

        // Always render the latest aircraft conditions.
        const Loads displayLoads =
            calculateLoads(
                state,
                config.aircraft,
                controls,
                config.environment
            );

        // Existing aircraft and background.
        renderer.draw(
            window,
            state,
            displayLoads,
            controls,
            simulationSeconds,
            paused
        );

        // Existing autopilot panel (aviation units).
        autopilotOverlay.draw(
            window,
            autopilot,
            state,
            paused
        );

        // Existing attitude indicator and altitude graph.
        flightInstruments.draw(
            window,
            state,
            telemetry
        );

        // NEW: Draw configuration screen on top.
        if (settingsScreen.isOpen()) {

            settingsScreen.draw(window);
        }

        window.display();
    }

    // ========================================================
    // 9. DISPLAY FINAL PERFORMANCE STATISTICS
    // ========================================================

    const auto& stats = telemetry.statistics();

    if (stats.engagedSeconds > 0.0) {

        std::cout
            << "\nAutopilot performance (latest run)\n"
            << "Engaged time: "
            << stats.engagedSeconds << " s\n"
            << "Mean absolute altitude error: "
            << stats.meanAbsoluteErrorMeters() *
                   kMetersToFeet
            << " ft\n"
            << "Maximum absolute altitude error: "
            << stats.maximumAbsoluteErrorMeters *
                   kMetersToFeet
            << " ft\n";
    }

    std::cout
        << "\nFlight simulation closed.\n";

    return 0;
}
