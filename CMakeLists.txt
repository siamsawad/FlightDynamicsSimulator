
# ============================================================
# FLIGHT DYNAMICS SIMULATOR - VERSION 13
# C++17 + SFML 3.1 + CMake
# ============================================================

cmake_minimum_required(VERSION 3.28)

project(FlightDynamicsSimulator LANGUAGES CXX)

# ============================================================
# 1. C++ LANGUAGE SETTINGS
# ============================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================
# 2. REUSABLE FLIGHT SIMULATION LIBRARY
# ============================================================

add_library(flight_core

    # Original 2D physics and controls.
    src/Aerodynamics.cpp
    src/PhysicsEngine.cpp
    src/ControlSystem.cpp

    # Original autopilot and telemetry.
    src/Autopilot.cpp
    src/Telemetry.cpp

    # Aircraft configuration system.
    src/SimulationConfig.cpp

    # Version 9: Flight-envelope and ground physics.
    src/FlightEnvelope.cpp
    src/GroundContact.cpp

    # Version 10: Six-degree-of-freedom dynamics.
    src/Dynamics6DOF.cpp

    # Version 11: 6-DOF controls and camera projection.
    src/Controls6DOF.cpp
    src/Projection3D.cpp

    # Version 12 and 13: 6-DOF autopilot.
    src/Autopilot6DOF.cpp
)

target_include_directories(flight_core PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

# ============================================================
# 3. ORIGINAL 2D CONSOLE SIMULATOR
# ============================================================

add_executable(flight_sim
    src/main.cpp
)

target_link_libraries(flight_sim PRIVATE
    flight_core
)

# ============================================================
# 4. 6-DOF CONSOLE SIMULATOR
# ============================================================

add_executable(flight_6dof
    src/flight_6dof.cpp
)

target_link_libraries(flight_6dof PRIVATE
    flight_core
)

# ============================================================
# 5. AUTOMATED 6-DOF FLIGHT SCENARIOS
# ============================================================

add_executable(flight_6dof_scenarios
    src/flight_6dof_scenarios.cpp
)

target_link_libraries(flight_6dof_scenarios PRIVATE
    flight_core
)

# ============================================================
# 6. AUTOMATED TESTS
# ============================================================

include(CTest)

if(BUILD_TESTING)

    # --------------------------------------------------------
    # TEST 1: FLIGHT PHYSICS
    # --------------------------------------------------------

    add_executable(flight_tests
        tests/test_physics.cpp
    )

    target_link_libraries(flight_tests PRIVATE
        flight_core
    )

    add_test(
        NAME FlightPhysicsTests
        COMMAND flight_tests
    )

    # --------------------------------------------------------
    # TEST 2: ORIGINAL FLIGHT CONTROLS
    # --------------------------------------------------------

    add_executable(flight_control_tests
        tests/test_controls.cpp
    )

    target_link_libraries(flight_control_tests PRIVATE
        flight_core
    )

    add_test(
        NAME FlightControlTests
        COMMAND flight_control_tests
    )

    # --------------------------------------------------------
    # TEST 3: ORIGINAL PID AUTOPILOT
    # --------------------------------------------------------

    add_executable(flight_autopilot_tests
        tests/test_autopilot.cpp
    )

    target_link_libraries(flight_autopilot_tests PRIVATE
        flight_core
    )

    add_test(
        NAME FlightAutopilotTests
        COMMAND flight_autopilot_tests
    )

    # --------------------------------------------------------
    # TEST 4: TELEMETRY
    # --------------------------------------------------------

    add_executable(flight_telemetry_tests
        tests/test_telemetry.cpp
    )

    target_link_libraries(flight_telemetry_tests PRIVATE
        flight_core
    )

    add_test(
        NAME FlightTelemetryTests
        COMMAND flight_telemetry_tests
    )

    # --------------------------------------------------------
    # TEST 5: CONFIGURATION
    # --------------------------------------------------------

    add_executable(flight_configuration_tests
        tests/test_configuration.cpp
    )

    target_link_libraries(
        flight_configuration_tests PRIVATE
        flight_core
    )

    add_test(
        NAME FlightConfigurationTests
        COMMAND flight_configuration_tests
    )

    # --------------------------------------------------------
    # TEST 6: FLIGHT SAFETY
    # --------------------------------------------------------

    add_executable(flight_safety_tests
        tests/test_safety.cpp
    )

    target_link_libraries(flight_safety_tests PRIVATE
        flight_core
    )

    add_test(
        NAME FlightSafetyTests
        COMMAND flight_safety_tests
    )

    # --------------------------------------------------------
    # TEST 7: FULL 6-DOF DYNAMICS
    # --------------------------------------------------------

    add_executable(flight_6dof_tests
        tests/test_6dof.cpp
    )

    target_link_libraries(flight_6dof_tests PRIVATE
        flight_core
    )

    add_test(
        NAME Flight6DOFTests
        COMMAND flight_6dof_tests
    )

    # --------------------------------------------------------
    # TEST 8: 6-DOF CONTROLS AND CAMERA
    # --------------------------------------------------------

    add_executable(flight_6dof_control_tests
        tests/test_6dof_controls.cpp
    )

    target_link_libraries(
        flight_6dof_control_tests PRIVATE
        flight_core
    )

    add_test(
        NAME Flight6DOFControlCameraTests
        COMMAND flight_6dof_control_tests
    )

    # --------------------------------------------------------
    # TEST 9: 6-DOF AUTOPILOT
    # --------------------------------------------------------

    add_executable(flight_6dof_autopilot_tests
        tests/test_autopilot_6dof.cpp
    )

    target_link_libraries(
        flight_6dof_autopilot_tests PRIVATE
        flight_core
    )

    add_test(
        NAME Flight6DOFAutopilotTests
        COMMAND flight_6dof_autopilot_tests
    )

endif()

# ============================================================
# 7. SFML 3.1 GRAPHICS LIBRARY
# ============================================================

set(BUILD_SHARED_LIBS OFF CACHE BOOL
    "Build static libraries" FORCE
)

set(SFML_BUILD_AUDIO OFF CACHE BOOL
    "Disable SFML audio" FORCE
)

set(SFML_BUILD_NETWORK OFF CACHE BOOL
    "Disable SFML networking" FORCE
)

include(FetchContent)

FetchContent_Declare(
    SFML
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG 3.1.0
    GIT_SHALLOW ON
    EXCLUDE_FROM_ALL
)

FetchContent_MakeAvailable(SFML)

# ============================================================
# 8. ORIGINAL 2D GRAPHICAL SIMULATOR
# ============================================================

add_executable(flight_gui

    src/flight_gui.cpp

    src/Renderer.cpp
    src/AutopilotOverlay.cpp
    src/FlightInstruments.cpp
    src/SettingsScreen.cpp
)

target_link_libraries(flight_gui PRIVATE
    flight_core
    SFML::Graphics
)

# ============================================================
# 9. 3D GRAPHICAL SIMULATOR - VERSION 13
# ============================================================

add_executable(flight_6dof_gui

    # Existing Version 12 application.
    src/flight_6dof_gui.cpp

    # Existing 3D aircraft renderer.
    src/Renderer6DOF.cpp

    # Existing configuration menu.
    src/SettingsScreen.cpp

    # Existing autopilot status overlay.
    src/Autopilot6DOFOverlay.cpp

    # NEW: Version 13 PFD and scenery.
    src/Visuals13.cpp
)

target_link_libraries(flight_6dof_gui PRIVATE
    flight_core
    SFML::Graphics
)

# ============================================================
# 10. COMPILER WARNINGS
# ============================================================

if(MSVC)

    foreach(target_name IN ITEMS
        flight_core
        flight_sim
        flight_6dof
        flight_6dof_scenarios
        flight_gui
        flight_6dof_gui
    )

        target_compile_options(
            ${target_name} PRIVATE /W4
        )

    endforeach()

else()

    foreach(target_name IN ITEMS
        flight_core
        flight_sim
        flight_6dof
        flight_6dof_scenarios
        flight_gui
        flight_6dof_gui
    )

        target_compile_options(
            ${target_name} PRIVATE
            -Wall
            -Wextra
            -Wpedantic
        )

    endforeach()

endif()
