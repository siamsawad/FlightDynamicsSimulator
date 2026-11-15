
#include "FlightSim/Autopilot6DOF.hpp"
#include "FlightSim/Autopilot6DOFOverlay.hpp"
#include "FlightSim/Controls6DOF.hpp"
#include "FlightSim/Dynamics6DOF.hpp"
#include "FlightSim/Renderer6DOF.hpp"
#include "FlightSim/SettingsScreen.hpp"
#include "FlightSim/SimulationConfig.hpp"

#include <SFML/Graphics.hpp>

#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>

int main() {

    using namespace flight;
    using namespace flight::sixdof;

    // ========================================================
    // 1. CREATE SFML WINDOW
    // ========================================================

    sf::RenderWindow window(
        sf::VideoMode({1280u, 720u}),
        "Flight Dynamics Simulator - Version 12 (3D Autopilot)",
        sf::Style::Titlebar | sf::Style::Close
    );

    window.setFramerateLimit(60);

    // Disable repeated keypress events while holding keys.
    // Continuous flight inputs use isKeyPressed().
    window.setKeyRepeatEnabled(false);

    // ========================================================
    // 2. EXISTING 6-DOF AIRCRAFT COMPONENTS
    // ========================================================

    SimulationConfig config;

    Model model;
    model.aircraft = config.aircraft;

    Wind wind = makeWind(config.environment);

    State state = makeInitialState(config);

    Inputs controls;

    const Controls6DOF actuator;

    // ========================================================
    // 3. GRAPHICAL COMPONENTS
    // ========================================================

    Renderer6DOF renderer;
    SettingsScreen settings;

    std::deque<Vec3> flightTrail;

    flightTrail.push_back(state.positionWorldMeters);

    // ========================================================
    // 4. NEW AUTOPILOT COMPONENTS
    // ========================================================

    Autopilot6DOF autopilot;
    Autopilot6DOFOverlay autopilotOverlay;

    // ========================================================
    // 5. FLIGHT TELEMETRY FILE
    // ========================================================

    // We preserve the Version 11 CSV filename, but
    // add new columns for autopilot measurements.
    std::ofstream csv(
        "flight_6dof_gui_telemetry.csv"
    );

    if (!csv) {
        std::cerr
            << "Warning: Cannot write 3D flight telemetry.\n";
    }

    csv << std::fixed
        << std::setprecision(6);

    csv
        << "run_id,time_s,north_m,east_m,down_m,"
        << "altitude_ft,airspeed_kts,"
        << "roll_deg,pitch_deg,yaw_deg,"
        << "p_radps,q_radps,r_radps,"
        << "aoa_deg,beta_deg,"
        << "throttle,elevator,aileron,rudder,"
        << "autopilot_engaged,target_altitude_ft,"
        << "altitude_error_ft,target_heading_deg,"
        << "heading_error_deg,low_airspeed,high_aoa\n";

    // ========================================================
    // 6. SIMULATION TIMING
    // ========================================================

    sf::Clock frameClock;

    double accumulator = 0.0;
    double simulationSeconds = 0.0;

    int physicsSteps = 0;
    int runId = 1;

    bool paused = false;
    bool pausedBeforeSettings = false;

    // ========================================================
    // 7. HELPER: RECORD FLIGHT DATA
    // ========================================================

    auto record = [&]() {

        if (!csv) {
            return;
        }

        const ForcesMoments loads =
            calculateForcesMoments(
                state,
                model,
                controls,
                wind
            );

        const EulerAngles angles =
            toEuler(state.bodyToWorld);

        const auto& ap = autopilot.status();

        const double headingError = std::remainder(
            ap.targetHeadingRadians - angles.yawRadians,
            2.0 * kPi
        );

        // ----------------------------------------------------
        // AIRCRAFT POSITION AND ORIENTATION
        // ----------------------------------------------------

        csv
            << runId << ','
            << simulationSeconds << ','

            << state.positionWorldMeters.x << ','
            << state.positionWorldMeters.y << ','
            << state.positionWorldMeters.z << ','

            << -state.positionWorldMeters.z /
               kFeetToMeters << ','

            << loads.air.trueAirspeedMetersPerSecond /
               kKnotsToMetersPerSecond << ','

            << degrees(angles.rollRadians) << ','
            << degrees(angles.pitchRadians) << ','
            << degrees(angles.yawRadians) << ','

            << state.angularVelocityBodyRadPerSecond.x
            << ','

            << state.angularVelocityBodyRadPerSecond.y
            << ','

            << state.angularVelocityBodyRadPerSecond.z
            << ','

            << degrees(
                loads.air.angleOfAttackRadians
            ) << ','

            << degrees(
                loads.air.sideslipRadians
            ) << ','

            << controls.throttle << ','
            << controls.elevator << ','
            << controls.aileron << ','
            << controls.rudder << ','

            << (ap.engaged ? 1 : 0) << ',';

        // ----------------------------------------------------
        // NEW: AUTOPILOT TRACKING DATA
        // ----------------------------------------------------

        if (ap.engaged) {

            csv
                << ap.targetAltitudeMeters /
                   kFeetToMeters << ','

                << (
                    ap.targetAltitudeMeters +
                    state.positionWorldMeters.z
                   ) /
                   kFeetToMeters << ','

                << degrees(
                    ap.targetHeadingRadians
                   ) << ','

                << degrees(headingError) << ',';

        } else {

            // Leave all four target/error fields empty
            // when autopilot is not engaged.
            csv << ",,,,";
        }

        csv
            << (
                ap.engaged && ap.lowAirspeed
                    ? 1
                    : 0
               ) << ','

            << (
                ap.engaged && ap.highAngleOfAttack
                    ? 1
                    : 0
               ) << '\n';

        // Flush periodically during the flight.
        if (physicsSteps % 120 == 0) {
            csv.flush();
        }
    };

    // ========================================================
    // 8. HELPER: RESTART FLIGHT
    // ========================================================

    auto restart = [&]() {

        // Rebuild the initial state using selected F2
        // aircraft and environmental settings.
        state = makeInitialState(config);

        model = Model{};
        model.aircraft = config.aircraft;

        wind = makeWind(config.environment);

        // Reset surfaces, throttle, and autopilot.
        controls = Inputs{};
        autopilot = Autopilot6DOF{};

        // Clear the flight path.
        flightTrail.clear();

        flightTrail.push_back(
            state.positionWorldMeters
        );

        // Reset the simulation clock.
        accumulator = 0.0;
        simulationSeconds = 0.0;
        physicsSteps = 0;

        paused = false;

        // Start another numbered CSV run.
        ++runId;

        record();

        std::cout
            << "3D flight restarted using "
            << "selected F2 configuration.\n";
    };

    record();

    std::cout
        << "Version 12: 6-DOF altitude and heading hold\n"
        << "H: Autopilot toggle\n"
        << "PgUp/PgDn: Altitude +/-100 ft\n"
        << "J/L: Target heading -/+10 degrees\n"
        << "UP/DOWN: Pitch\n"
        << "A/D or LEFT/RIGHT: Roll\n"
        << "Q/E: Rudder\n"
        << "W/S: Throttle\n"
        << "F2: Settings | P: Pause | R: Restart\n"
        << "Manual surfaces override autopilot.\n";

    // ========================================================
    // 9. MAIN APPLICATION LOOP
    // ========================================================

    while (window.isOpen()) {

        bool touchedSettings = false;

        // ----------------------------------------------------
        // A. PROCESS SFML EVENTS
        // ----------------------------------------------------

        while (const auto event = window.pollEvent()) {

            if (event->is<sf::Event::Closed>()) {

                window.close();
                break;
            }

            // -----------------------------------------------
            // SETTINGS SCREEN
            // -----------------------------------------------

            if (settings.isOpen()) {

                touchedSettings = true;

                SimulationConfig candidate = config;

                const bool applied =
                    settings.handleEvent(
                        *event,
                        candidate
                    );

                if (applied) {

                    config = candidate;

                    restart();

                    std::cout
                        << "New 3D configuration applied.\n";

                } else if (!settings.isOpen()) {

                    // Restore the previous pause state
                    // if the user cancels F2 settings.
                    paused = pausedBeforeSettings;

                    accumulator = 0.0;
                }

                continue;
            }

            // -----------------------------------------------
            // KEYBOARD COMMANDS
            // -----------------------------------------------

            if (const auto* key =
                    event->getIf<sf::Event::KeyPressed>()) {

                const auto code = key->code;

                // F2: Open aircraft configuration.
                if (code == sf::Keyboard::Key::F2) {

                    pausedBeforeSettings = paused;

                    paused = true;
                    accumulator = 0.0;

                    settings.open(config);

                    touchedSettings = true;
                }

                // Escape: Exit.
                else if (
                    code == sf::Keyboard::Key::Escape
                ) {

                    window.close();
                }

                // P: Pause/resume.
                else if (
                    code == sf::Keyboard::Key::P
                ) {

                    paused = !paused;
                }

                // R: Restart current configuration.
                else if (
                    code == sf::Keyboard::Key::R
                ) {

                    restart();
                }

                // -------------------------------------------
                // H: TOGGLE NEW 3D AUTOPILOT
                // -------------------------------------------

                else if (
                    !paused &&
                    code == sf::Keyboard::Key::H
                ) {

                    if (autopilot.isEngaged()) {

                        autopilot.disengage();

                        std::cout
                            << "3D autopilot OFF.\n";

                    } else {

                        // First capture current conditions.
                        autopilot.engage(state);

                        // Then use the target altitude from
                        // the existing F2 configuration.
                        autopilot.setTargetAltitudeMeters(
                            config.autopilotTargetFeet *
                            kFeetToMeters
                        );

                        // Current heading becomes target.
                        std::cout
                            << "3D autopilot ON.\n";
                    }
                }

                // -------------------------------------------
                // TARGET ALTITUDE AND HEADING ADJUSTMENTS
                // -------------------------------------------

                else if (
                    !paused &&
                    autopilot.isEngaged()
                ) {

                    if (
                        code ==
                        sf::Keyboard::Key::PageUp
                    ) {

                        autopilot.adjustTargetAltitudeMeters(
                            +100.0 * kFeetToMeters
                        );
                    }

                    else if (
                        code ==
                        sf::Keyboard::Key::PageDown
                    ) {

                        autopilot.adjustTargetAltitudeMeters(
                            -100.0 * kFeetToMeters
                        );
                    }

                    // J: Turn left 10 degrees.
                    else if (
                        code == sf::Keyboard::Key::J
                    ) {

                        autopilot.adjustTargetHeadingRadians(
                            radians(-10.0)
                        );
                    }

                    // L: Turn right 10 degrees.
                    else if (
                        code == sf::Keyboard::Key::L
                    ) {

                        autopilot.adjustTargetHeadingRadians(
                            radians(+10.0)
                        );
                    }
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

        // ----------------------------------------------------
        // C. READ CONTINUOUS PILOT INPUTS
        // ----------------------------------------------------

        PilotAxes pilot;

        if (
            !settings.isOpen() &&
            !touchedSettings &&
            window.hasFocus()
        ) {

            const auto down =
                [](sf::Keyboard::Key key) {

                    return sf::Keyboard::isKeyPressed(
                        key
                    );
                };

            // Pitch.
            if (down(sf::Keyboard::Key::Up)) {
                pilot.pitch += 1.0;
            }

            if (down(sf::Keyboard::Key::Down)) {
                pilot.pitch -= 1.0;
            }

            // Roll.
            if (
                down(sf::Keyboard::Key::D) ||
                down(sf::Keyboard::Key::Right)
            ) {

                pilot.roll += 1.0;
            }

            if (
                down(sf::Keyboard::Key::A) ||
                down(sf::Keyboard::Key::Left)
            ) {

                pilot.roll -= 1.0;
            }

            // Yaw / Rudder.
            if (down(sf::Keyboard::Key::E)) {
                pilot.yaw += 1.0;
            }

            if (down(sf::Keyboard::Key::Q)) {
                pilot.yaw -= 1.0;
            }

            // Throttle remains manual.
            if (down(sf::Keyboard::Key::W)) {
                pilot.throttle += 1.0;
            }

            if (down(sf::Keyboard::Key::S)) {
                pilot.throttle -= 1.0;
            }

            // -----------------------------------------------
            // MANUAL OVERRIDE
            // -----------------------------------------------

            // Pilot use of elevator, ailerons, or rudder
            // immediately disengages altitude/heading hold.
            //
            // Throttle input alone does NOT disengage AP.
            if (
                autopilot.isEngaged() &&
                (
                    pilot.pitch != 0.0 ||
                    pilot.roll != 0.0 ||
                    pilot.yaw != 0.0
                )
            ) {

                autopilot.disengage();

                std::cout
                    << "Manual override: autopilot OFF.\n";
            }
        }

        // ====================================================
        // 10. FIXED-TIMESTEP 6-DOF PHYSICS
        // ====================================================

        if (
            !paused &&
            !settings.isOpen() &&
            !touchedSettings &&
            window.hasFocus()
        ) {

            accumulator += frameSeconds;

            while (accumulator >= kTimeStep) {

                // --------------------------------------------
                // STEP 1: BEGIN WITH MANUAL INPUT
                // --------------------------------------------

                PilotAxes commanded = pilot;

                // --------------------------------------------
                // STEP 2: AUTOPILOT FEEDBACK
                // --------------------------------------------

                if (autopilot.isEngaged()) {

                    // Measure airspeed, AoA, and sideslip
                    // from the actual 6-DOF physics model.
                    const ForcesMoments loadsBefore =
                        calculateForcesMoments(
                            state,
                            model,
                            controls,
                            wind
                        );

                    // Request elevator, ailerons, rudder.
                    const PilotAxes apRequest =
                        autopilot.update(
                            state,
                            loadsBefore.air,
                            model,
                            kTimeStep
                        );

                    commanded.pitch = apRequest.pitch;
                    commanded.roll = apRequest.roll;
                    commanded.yaw = apRequest.yaw;

                    // Do not replace commanded.throttle.
                    // W/S remain under pilot control.
                }

                // --------------------------------------------
                // STEP 3: RATE-LIMIT CONTROL SURFACES
                // --------------------------------------------

                actuator.update(
                    controls,
                    commanded,
                    kTimeStep
                );

                // --------------------------------------------
                // STEP 4: ADVANCE REAL 6-DOF PHYSICS
                // --------------------------------------------

                step(
                    state,
                    model,
                    controls,
                    wind,
                    kTimeStep
                );

                // --------------------------------------------
                // STEP 5: ADVANCE SIMULATION CLOCK
                // --------------------------------------------

                accumulator -= kTimeStep;
                simulationSeconds += kTimeStep;

                ++physicsSteps;

                // --------------------------------------------
                // STEP 6: RECORD AT 10 HZ
                // --------------------------------------------

                if (physicsSteps % 12 == 0) {

                    flightTrail.push_back(
                        state.positionWorldMeters
                    );

                    if (flightTrail.size() > 250) {
                        flightTrail.pop_front();
                    }

                    record();
                }
            }

        } else {

            accumulator = 0.0;
        }

        // ====================================================
        // 11. RENDER THE 3D SIMULATOR
        // ====================================================

        const ForcesMoments loads =
            calculateForcesMoments(
                state,
                model,
                controls,
                wind
            );

        // Existing aircraft scene and flight instruments.
        renderer.draw(
            window,
            state,
            loads,
            controls,
            flightTrail,
            simulationSeconds,
            paused || settings.isOpen()
        );

        // NEW: Altitude and heading hold panel.
        autopilotOverlay.draw(
            window,
            autopilot,
            state,
            paused || settings.isOpen()
        );

        // Existing F2 settings screen appears last.
        if (settings.isOpen()) {
            settings.draw(window);
        }

        window.display();
    }

    std::cout
        << "3D flight session finished.\n";

    return 0;
}
