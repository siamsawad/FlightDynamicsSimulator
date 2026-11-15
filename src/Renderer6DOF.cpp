
#include "FlightSim/Renderer6DOF.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <vector>

namespace flight::sixdof {

namespace {

// ============================================================
// 1. COLORS AND FORMATTING
// ============================================================

const sf::Color kText(239, 246, 255);
const sf::Color kMuted(168, 197, 217);
const sf::Color kAccent(92, 214, 241);
const sf::Color kGreen(115, 232, 163);

std::string number(double value, int decimals = 1) {

    std::ostringstream out;

    out << std::fixed
        << std::setprecision(decimals)
        << value;

    return out.str();
}

// ============================================================
// 2. DRAWING HELPERS
// ============================================================

void line(
    sf::RenderWindow& window,
    sf::Vector2f a,
    sf::Vector2f b,
    sf::Color color
) {

    const sf::Vertex vertices[] = {
        {a, color},
        {b, color}
    };

    window.draw(
        vertices,
        2,
        sf::PrimitiveType::Lines
    );
}

void panel(
    sf::RenderWindow& window,
    sf::Vector2f at,
    sf::Vector2f dimensions
) {

    sf::RectangleShape shape(dimensions);

    shape.setPosition(at);

    shape.setFillColor(
        sf::Color(12, 26, 43, 228)
    );

    shape.setOutlineColor(
        sf::Color(89, 133, 164)
    );

    shape.setOutlineThickness(1.0f);

    window.draw(shape);
}

// ============================================================
// 3. AIRCRAFT MESH
// ============================================================

// Aircraft BODY coordinates:
// +X = forward, +Y = right, +Z = down.
struct LocalTriangle {

    Vec3 a;
    Vec3 b;
    Vec3 c;

    sf::Color color;
};

struct ScreenTriangle {

    std::array<sf::Vector2f, 3> points;

    sf::Color color;

    double depth = 0.0;
};

// Original simple aircraft model.
const std::vector<LocalTriangle>& aircraftMesh() {

    static const std::vector<LocalTriangle> mesh = [] {

        const sf::Color bodyTop(225, 234, 244);
        const sf::Color bodySide(151, 178, 200);

        const sf::Color wingTop(96, 178, 209);
        const sf::Color wingBottom(56, 131, 168);

        const sf::Color tail(224, 181, 107);

        return std::vector<LocalTriangle>{

            // ------------------------------------------------
            // FUSELAGE
            // ------------------------------------------------

            {
                {5.3, 0, 0},
                {0.9, -0.55, -0.08},
                {0.9, 0, -0.55},
                bodyTop
            },

            {
                {5.3, 0, 0},
                {0.9, 0, -0.55},
                {0.9, 0.55, -0.08},
                bodyTop
            },

            {
                {0.9, -0.55, -0.08},
                {-3.8, -0.19, 0.02},
                {0.9, 0, -0.55},
                bodySide
            },

            {
                {0.9, 0, -0.55},
                {-3.8, 0.19, 0.02},
                {0.9, 0.55, -0.08},
                bodySide
            },

            {
                {0.9, -0.55, -0.08},
                {0.9, 0.55, -0.08},
                {-3.8, 0.19, 0.02},
                bodySide
            },

            {
                {0.9, -0.55, -0.08},
                {-3.8, 0.19, 0.02},
                {-3.8, -0.19, 0.02},
                bodySide
            },

            // ------------------------------------------------
            // LEFT MAIN WING
            // ------------------------------------------------

            {
                {1.05, -0.37, 0.01},
                {-0.35, -5.7, 0.18},
                {-1.2, -5.7, 0.18},
                wingTop
            },

            {
                {1.05, -0.37, 0.01},
                {-1.2, -5.7, 0.18},
                {-1.4, -0.37, 0.01},
                wingBottom
            },

            // ------------------------------------------------
            // RIGHT MAIN WING
            // ------------------------------------------------

            {
                {1.05, 0.37, 0.01},
                {-1.2, 5.7, 0.18},
                {-0.35, 5.7, 0.18},
                wingTop
            },

            {
                {1.05, 0.37, 0.01},
                {-1.4, 0.37, 0.01},
                {-1.2, 5.7, 0.18},
                wingBottom
            },

            // ------------------------------------------------
            // LEFT HORIZONTAL STABILIZER
            // ------------------------------------------------

            {
                {-3.1, -0.15, 0.01},
                {-3.4, -2, 0.05},
                {-4.2, -2, 0.05},
                tail
            },

            {
                {-3.1, -0.15, 0.01},
                {-4.2, -2, 0.05},
                {-4.2, -0.15, 0.01},
                tail
            },

            // ------------------------------------------------
            // RIGHT HORIZONTAL STABILIZER
            // ------------------------------------------------

            {
                {-3.1, 0.15, 0.01},
                {-4.2, 2, 0.05},
                {-3.4, 2, 0.05},
                tail
            },

            {
                {-3.1, 0.15, 0.01},
                {-4.2, 0.15, 0.01},
                {-4.2, 2, 0.05},
                tail
            },

            // ------------------------------------------------
            // VERTICAL STABILIZER
            // ------------------------------------------------

            {
                {-3.3, 0, 0},
                {-4.05, 0, -1.9},
                {-4.3, 0, 0},
                tail
            }
        };
    }();

    return mesh;
}

} // anonymous namespace

// ============================================================
// 4. RENDERER CONSTRUCTOR
// ============================================================

Renderer6DOF::Renderer6DOF() {

    fontLoaded_ = font_.openFromFile(
        "C:/Windows/Fonts/segoeui.ttf"
    );

    if (!fontLoaded_) {

        fontLoaded_ = font_.openFromFile(
            "C:/Windows/Fonts/arial.ttf"
        );
    }
}

// ============================================================
// 5. TEXT DRAWING HELPER
// ============================================================

void Renderer6DOF::text(
    sf::RenderWindow& window,
    const std::string& value,
    unsigned int size,
    sf::Vector2f position,
    sf::Color color
) const {

    if (!fontLoaded_) {
        return;
    }

    sf::Text label(font_, value, size);

    label.setPosition(position);
    label.setFillColor(color);

    window.draw(label);
}

// ============================================================
// 6. SCENERY, SKY, GROUND, AND FLIGHT TRAIL
// ============================================================

void Renderer6DOF::drawScene(
    sf::RenderWindow& window,
    const State& state,
    const Camera3D& camera,
    const std::deque<Vec3>& trail
) const {

    const float width =
        static_cast<float>(window.getSize().x);

    const float height =
        static_cast<float>(window.getSize().y);

    // Clear background to blue sky.
    window.clear(
        sf::Color(27, 62, 101)
    );

    // --------------------------------------------------------
    // A. PROJECT REFERENCE HORIZON
    // --------------------------------------------------------

    const Vec3 horizontalHeading{
        camera.forwardWorld.x,
        camera.forwardWorld.y,
        0.0
    };

    const Vec3 farPoint =
        state.positionWorldMeters
        + horizontalHeading * 5000.0;

    const auto far =
        projectPoint(camera, farPoint);

    const float horizonY =
        far.inFront
            ? std::clamp(
                static_cast<float>(far.yPixels),
                0.0f,
                height
              )
            : height * 0.35f;

    // --------------------------------------------------------
    // B. GROUND BACKGROUND
    // --------------------------------------------------------

    sf::RectangleShape lower({
        width,
        height - horizonY
    });

    lower.setPosition({
        0.0f,
        horizonY
    });

    lower.setFillColor(
        sf::Color(65, 99, 115)
    );

    window.draw(lower);

    // --------------------------------------------------------
    // C. VERSION 13 ENVIRONMENT
    // --------------------------------------------------------

    visuals13_.drawEnvironment(
        window,
        state,
        camera,
        horizonY
    );

    // Draw reference horizon line above scenery.
    line(
        window,
        {0.0f, horizonY},
        {width, horizonY},
        sf::Color(150, 207, 225)
    );

    // --------------------------------------------------------
    // D. FLAT SEA-LEVEL GROUND GRID
    // --------------------------------------------------------

    // Visual reference only; no 3D collision.
    const double altitude =
        -state.positionWorldMeters.z;

    if (altitude < 350.0) {

        const double cx =
            std::floor(
                state.positionWorldMeters.x / 50.0
            ) * 50.0;

        const double cy =
            std::floor(
                state.positionWorldMeters.y / 50.0
            ) * 50.0;

        for (int i = -8; i <= 8; ++i) {

            const double offset =
                i * 50.0;

            // Grid lines parallel to world Y.
            const auto a = projectPoint(
                camera,
                {cx + offset, cy - 400.0, 0.0}
            );

            const auto b = projectPoint(
                camera,
                {cx + offset, cy + 400.0, 0.0}
            );

            if (a.inFront && b.inFront) {

                line(
                    window,
                    {
                        static_cast<float>(a.xPixels),
                        static_cast<float>(a.yPixels)
                    },
                    {
                        static_cast<float>(b.xPixels),
                        static_cast<float>(b.yPixels)
                    },
                    sf::Color(122, 159, 150, 100)
                );
            }

            // Grid lines parallel to world X.
            const auto c = projectPoint(
                camera,
                {cx - 400.0, cy + offset, 0.0}
            );

            const auto d = projectPoint(
                camera,
                {cx + 400.0, cy + offset, 0.0}
            );

            if (c.inFront && d.inFront) {

                line(
                    window,
                    {
                        static_cast<float>(c.xPixels),
                        static_cast<float>(c.yPixels)
                    },
                    {
                        static_cast<float>(d.xPixels),
                        static_cast<float>(d.yPixels)
                    },
                    sf::Color(122, 159, 150, 100)
                );
            }
        }
    }

    // --------------------------------------------------------
    // E. EXISTING 3D FLIGHT PATH
    // --------------------------------------------------------

    for (
        std::size_t i = 1;
        i < trail.size();
        ++i
    ) {

        const auto a =
            projectPoint(camera, trail[i - 1]);

        const auto b =
            projectPoint(camera, trail[i]);

        if (!a.inFront || !b.inFront) {
            continue;
        }

        line(
            window,
            {
                static_cast<float>(a.xPixels),
                static_cast<float>(a.yPixels)
            },
            {
                static_cast<float>(b.xPixels),
                static_cast<float>(b.yPixels)
            },
            sf::Color(106, 235, 218, 170)
        );
    }
}

// ============================================================
// 7. DRAW THE 3D AIRCRAFT
// ============================================================

void Renderer6DOF::drawAircraft(
    sf::RenderWindow& window,
    const State& state,
    const Camera3D& camera
) const {

    std::vector<ScreenTriangle> triangles;

    triangles.reserve(
        aircraftMesh().size()
    );

    // --------------------------------------------------------
    // A. TRANSFORM EACH TRIANGLE
    // --------------------------------------------------------

    for (const LocalTriangle& face : aircraftMesh()) {

        // Rotate body vertices by the actual aircraft
        // quaternion, then translate into world space.
        const Vec3 a =
            state.positionWorldMeters
            + rotateBodyToWorld(
                state.bodyToWorld,
                face.a
            );

        const Vec3 b =
            state.positionWorldMeters
            + rotateBodyToWorld(
                state.bodyToWorld,
                face.b
            );

        const Vec3 c =
            state.positionWorldMeters
            + rotateBodyToWorld(
                state.bodyToWorld,
                face.c
            );

        // ----------------------------------------------------
        // B. PROJECT TO SCREEN SPACE
        // ----------------------------------------------------

        const auto pa =
            projectPoint(camera, a);

        const auto pb =
            projectPoint(camera, b);

        const auto pc =
            projectPoint(camera, c);

        if (
            !pa.inFront ||
            !pb.inFront ||
            !pc.inFront
        ) {

            continue;
        }

        ScreenTriangle screen;

        screen.points[0] = {
            static_cast<float>(pa.xPixels),
            static_cast<float>(pa.yPixels)
        };

        screen.points[1] = {
            static_cast<float>(pb.xPixels),
            static_cast<float>(pb.yPixels)
        };

        screen.points[2] = {
            static_cast<float>(pc.xPixels),
            static_cast<float>(pc.yPixels)
        };

        screen.color = face.color;

        screen.depth =
            (
                pa.depthMeters +
                pb.depthMeters +
                pc.depthMeters
            ) / 3.0;

        triangles.push_back(screen);
    }

    // --------------------------------------------------------
    // C. SORT DISTANT FACES FIRST
    // --------------------------------------------------------

    std::sort(
        triangles.begin(),
        triangles.end(),
        [](
            const ScreenTriangle& a,
            const ScreenTriangle& b
        ) {

            return a.depth > b.depth;
        }
    );

    // --------------------------------------------------------
    // D. DRAW THE PROJECTED MESH
    // --------------------------------------------------------

    for (const auto& triangle : triangles) {

        sf::ConvexShape shape(3);

        for (std::size_t i = 0; i < 3; ++i) {

            shape.setPoint(
                i,
                triangle.points[i]
            );
        }

        shape.setFillColor(
            triangle.color
        );

        shape.setOutlineThickness(0.7f);

        shape.setOutlineColor(
            sf::Color(26, 49, 69, 170)
        );

        window.draw(shape);
    }
}

// ============================================================
// 8. EXISTING FLIGHT HUD
// ============================================================

void Renderer6DOF::drawHUD(
    sf::RenderWindow& window,
    const State& state,
    const ForcesMoments& loads,
    const Inputs& controls,
    double timeSeconds,
    bool paused
) const {

    const EulerAngles a =
        toEuler(state.bodyToWorld);

    const double altitudeFt =
        -state.positionWorldMeters.z
        / kFeetToMeters;

    const double speedKt =
        loads.air.trueAirspeedMetersPerSecond
        / kKnotsToMetersPerSecond;

    const double verticalFtMin =
        -state.velocityWorldMetersPerSecond.z
        * 60.0 / kFeetToMeters;

    double heading =
        std::fmod(
            degrees(a.yawRadians),
            360.0
        );

    if (heading < 0.0) {
        heading += 360.0;
    }

    const float width =
        static_cast<float>(window.getSize().x);

    const float height =
        static_cast<float>(window.getSize().y);

    // --------------------------------------------------------
    // A. LEFT FLIGHT DATA PANEL
    // --------------------------------------------------------

    panel(
        window,
        {16.0f, 16.0f},
        {275.0f, 228.0f}
    );

    text(
        window,
        "6-DOF FLIGHT DATA",
        18,
        {28.0f, 23.0f},
        kAccent
    );

    text(
        window,
        "ALTITUDE     "
            + number(altitudeFt, 0)
            + " ft",
        16,
        {28.0f, 62.0f},
        kText
    );

    text(
        window,
        "TRUE AIRSPD  "
            + number(speedKt, 1)
            + " kt",
        16,
        {28.0f, 90.0f},
        kText
    );

    text(
        window,
        "VERT SPEED   "
            + number(verticalFtMin, 0)
            + " ft/min",
        15,
        {28.0f, 118.0f},
        kText
    );

    text(
        window,
        "AOA          "
            + number(
                degrees(loads.air.angleOfAttackRadians),
                1
              )
            + " deg",
        15,
        {28.0f, 147.0f},
        kText
    );

    text(
        window,
        "SIDESLIP     "
            + number(
                degrees(loads.air.sideslipRadians),
                1
              )
            + " deg",
        15,
        {28.0f, 176.0f},
        kText
    );

    text(
        window,
        loads.air.nominalStall
            ? "STALL / HIGH AOA"
            : "AIR DATA: NORMAL",
        13,
        {28.0f, 208.0f},
        loads.air.nominalStall
            ? sf::Color(255, 148, 105)
            : kGreen
    );

    // --------------------------------------------------------
    // B. RIGHT ATTITUDE / HEADING PANEL
    // --------------------------------------------------------

    panel(
        window,
        {width - 288.0f, 16.0f},
        {272.0f, 228.0f}
    );

    const float x =
        width - 275.0f;

    text(
        window,
        "ATTITUDE / HEADING",
        18,
        {x, 23.0f},
        kAccent
    );

    text(
        window,
        "ROLL       "
            + number(degrees(a.rollRadians), 1)
            + " deg",
        16,
        {x, 62.0f},
        kText
    );

    text(
        window,
        "PITCH      "
            + number(degrees(a.pitchRadians), 1)
            + " deg",
        16,
        {x, 92.0f},
        kText
    );

    text(
        window,
        "YAW        "
            + number(degrees(a.yawRadians), 1)
            + " deg",
        16,
        {x, 122.0f},
        kText
    );

    text(
        window,
        "HEADING    "
            + number(heading, 0)
            + " deg",
        16,
        {x, 152.0f},
        kText
    );

    text(
        window,
        "TIME       "
            + number(timeSeconds, 1)
            + " s",
        16,
        {x, 182.0f},
        kText
    );

    text(
        window,
        paused ? "PAUSED" : "6-DOF FLIGHT",
        13,
        {x, 210.0f},
        paused
            ? sf::Color(255, 196, 92)
            : kGreen
    );

    // --------------------------------------------------------
    // C. BOTTOM KEYBOARD / CONTROLS PANEL
    // --------------------------------------------------------

    panel(
        window,
        {16.0f, height - 117.0f},
        {width - 32.0f, 101.0f}
    );

    text(
        window,
        "CONTROLS: UP/DOWN Pitch   A/D or LEFT/RIGHT Roll   Q/E Rudder   W/S Throttle",
        16,
        {30.0f, height - 106.0f},
        kText
    );

    text(
        window,
        "H Autopilot  PgUp/PgDn Alt  J/L Heading  F2 Settings  P Pause  R Restart  ESC Exit",
        14,
        {30.0f, height - 76.0f},
        kMuted
    );

    text(
        window,
        "THROTTLE "
            + number(
                controls.throttle * 100.0,
                0
              )
            + "%   ELEV "
            + number(controls.elevator, 2)
            + "   AIL "
            + number(controls.aileron, 2)
            + "   RUD "
            + number(controls.rudder, 2),
        14,
        {30.0f, height - 48.0f},
        kAccent
    );
}

// ============================================================
// 9. DRAW THE COMPLETE FRAME
// ============================================================

void Renderer6DOF::draw(
    sf::RenderWindow& window,
    const State& state,
    const ForcesMoments& loads,
    const Inputs& controls,
    const std::deque<Vec3>& flightTrail,
    double simulationSeconds,
    bool paused
) const {

    // Create the chase camera.
    const Camera3D camera =
        makeChaseCamera(
            state,
            static_cast<double>(
                window.getSize().x
            ),
            static_cast<double>(
                window.getSize().y
            )
        );

    // Draw in the correct order.
    drawScene(
        window,
        state,
        camera,
        flightTrail
    );

    drawAircraft(
        window,
        state,
        camera
    );

    drawHUD(
        window,
        state,
        loads,
        controls,
        simulationSeconds,
        paused
    );

    // Version 13: Draw the artificial horizon / PFD.
    visuals13_.drawPFD(
        window,
        state,
        loads
    );
}

} // namespace flight::sixdof
