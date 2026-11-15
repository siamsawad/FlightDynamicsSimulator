
#include "FlightSim/Visuals13.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <vector>

namespace flight::sixdof {

namespace {

using Point = sf::Vector2f;

// ============================================================
// 1. GENERAL HELPERS
// ============================================================

std::string number(double value, int decimals = 0) {

    std::ostringstream stream;

    stream << std::fixed
           << std::setprecision(decimals)
           << value;

    return stream.str();
}

void drawLine(
    sf::RenderWindow& window,
    Point a,
    Point b,
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

void drawPolygon(
    sf::RenderWindow& window,
    const std::vector<Point>& vertices,
    sf::Color color
) {

    if (vertices.size() < 3) {
        return;
    }

    sf::ConvexShape polygon(vertices.size());

    for (std::size_t i = 0; i < vertices.size(); ++i) {
        polygon.setPoint(i, vertices[i]);
    }

    polygon.setFillColor(color);

    window.draw(polygon);
}

// ============================================================
// 2. ARTIFICIAL HORIZON CLIPPING
// ============================================================

// Signed distance relative to the moving horizon.
//
// Negative = sky side.
// Positive = ground side.
float horizonSide(
    Point point,
    Point horizon,
    Point normal
) {

    return
        (point.x - horizon.x) * normal.x +
        (point.y - horizon.y) * normal.y;
}

// Clip the blue or brown horizon polygon to the PFD
// rectangle using Sutherland-Hodgman clipping.
std::vector<Point> clipHorizon(
    const std::vector<Point>& polygon,
    Point horizon,
    Point normal,
    bool keepSky
) {

    std::vector<Point> result;

    if (polygon.empty()) {
        return result;
    }

    for (std::size_t i = 0; i < polygon.size(); ++i) {

        const Point a = polygon[i];

        const Point b =
            polygon[(i + 1) % polygon.size()];

        const float da =
            horizonSide(a, horizon, normal);

        const float db =
            horizonSide(b, horizon, normal);

        const bool aInside =
            keepSky ? da <= 0.0f : da >= 0.0f;

        const bool bInside =
            keepSky ? db <= 0.0f : db >= 0.0f;

        const auto intersection = [&]() {

            const float denominator = da - db;

            const float t =
                std::abs(denominator) > 1e-6f
                    ? da / denominator
                    : 0.0f;

            return Point{
                a.x + t * (b.x - a.x),
                a.y + t * (b.y - a.y)
            };
        };

        if (aInside && bInside) {

            result.push_back(b);

        } else if (aInside && !bInside) {

            result.push_back(intersection());

        } else if (!aInside && bInside) {

            result.push_back(intersection());
            result.push_back(b);
        }
    }

    return result;
}

// ============================================================
// 3. LINE CLIPPING
// ============================================================

// Liang-Barsky clipping.
//
// Prevents horizon and pitch-ladder lines from extending
// beyond the attitude indicator's rectangular viewport.
bool clipLine(
    Point& a,
    Point& b,
    float left,
    float top,
    float right,
    float bottom
) {

    const float dx = b.x - a.x;
    const float dy = b.y - a.y;

    float t0 = 0.0f;
    float t1 = 1.0f;

    const auto clip = [&](float p, float q) {

        if (std::abs(p) < 1e-7f) {
            return q >= 0.0f;
        }

        const float r = q / p;

        if (p < 0.0f) {

            if (r > t1) {
                return false;
            }

            t0 = std::max(t0, r);

        } else {

            if (r < t0) {
                return false;
            }

            t1 = std::min(t1, r);
        }

        return true;
    };

    if (
        !clip(-dx, a.x - left) ||
        !clip(dx, right - a.x) ||
        !clip(-dy, a.y - top) ||
        !clip(dy, bottom - a.y)
    ) {

        return false;
    }

    const Point start = a;

    a = {
        start.x + t0 * dx,
        start.y + t0 * dy
    };

    b = {
        start.x + t1 * dx,
        start.y + t1 * dy
    };

    return true;
}

// ============================================================
// 4. CLOUD SHAPES
// ============================================================

void drawCloud(
    sf::RenderWindow& window,
    float x,
    float y,
    float radius
) {

    struct Puff {
        float x;
        float y;
        float scale;
    };

    const Puff puffs[] = {
        {-1.1f,  0.10f, 0.78f},
        {-0.5f, -0.27f, 1.00f},
        { 0.2f, -0.45f, 1.10f},
        { 0.9f,  0.02f, 0.84f}
    };

    for (const Puff& puff : puffs) {

        const float r = radius * puff.scale;

        sf::CircleShape circle(r, 24);

        circle.setPosition({
            x + puff.x * radius - r,
            y + puff.y * radius - r
        });

        circle.setFillColor(
            sf::Color(239, 247, 255, 105)
        );

        window.draw(circle);
    }
}

} // anonymous namespace

// ============================================================
// 5. CONSTRUCTOR
// ============================================================

Visuals13::Visuals13() {

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
// 6. TEXT DRAWING
// ============================================================

void Visuals13::drawText(
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
// 7. ENVIRONMENT VISUALS
// ============================================================

void Visuals13::drawEnvironment(
    sf::RenderWindow& window,
    const State& state,
    const Camera3D& camera,
    float horizonY
) const {

    const float width =
        static_cast<float>(window.getSize().x);

    const float height =
        static_cast<float>(window.getSize().y);

    horizonY = std::clamp(
        horizonY,
        0.0f,
        height
    );

    // --------------------------------------------------------
    // A. SOFT SKY COLOR BANDS
    // --------------------------------------------------------

    for (int i = 0; i < 4; ++i) {

        const float top =
            horizonY * i / 4.0f;

        sf::RectangleShape band({
            width,
            horizonY / 4.0f + 1.0f
        });

        band.setPosition({0.0f, top});

        band.setFillColor(
            sf::Color(
                100,
                static_cast<std::uint8_t>(145 + i * 10),
                203,
                static_cast<std::uint8_t>(18 + i * 8)
            )
        );

        window.draw(band);
    }

    // --------------------------------------------------------
    // B. SOFT GROUND COLOR BANDS
    // --------------------------------------------------------

    for (int i = 0; i < 4; ++i) {

        const float groundHeight =
            height - horizonY;

        sf::RectangleShape band({
            width,
            groundHeight / 4.0f + 1.0f
        });

        band.setPosition({
            0.0f,
            horizonY + groundHeight * i / 4.0f
        });

        band.setFillColor(
            sf::Color(
                53,
                static_cast<std::uint8_t>(110 - i * 7),
                78,
                static_cast<std::uint8_t>(80 + i * 13)
            )
        );

        window.draw(band);
    }

    // --------------------------------------------------------
    // C. DISTANT HILLS
    // --------------------------------------------------------

    // These are decorative shapes, not collision terrain.
    for (int i = 0; i < 10; ++i) {

        const float x =
            i * width / 9.0f;

        const float peak =
            horizonY -
            10.0f -
            9.0f * std::sin(i * 1.7f);

        drawPolygon(
            window,
            {
                {x - width / 8.0f, horizonY + 25.0f},
                {x, peak},
                {x + width / 8.0f, horizonY + 25.0f}
            },
            sf::Color(76, 119, 122, 155)
        );
    }

    // --------------------------------------------------------
    // D. WORLD-SPACE CLOUDS
    // --------------------------------------------------------

    // NED coordinates:
    // Negative Z means altitude above sea level.
    const double altitude =
        std::max(
            0.0,
            -state.positionWorldMeters.z
        );

    const double cloudAltitude =
        std::floor(altitude / 500.0) * 500.0
        + 200.0;

    constexpr double spacingX = 650.0;
    constexpr double spacingY = 460.0;

    const double originX =
        std::floor(
            state.positionWorldMeters.x / spacingX
        ) * spacingX;

    const double originY =
        std::floor(
            state.positionWorldMeters.y / spacingY
        ) * spacingY;

    for (int ix = -2; ix <= 3; ++ix) {

        for (int iy = -3; iy <= 3; ++iy) {

            const Vec3 cloudWorld{
                originX + ix * spacingX + 330.0,
                originY + iy * spacingY + 180.0,
                -cloudAltitude
            };

            const auto point =
                projectPoint(camera, cloudWorld);

            if (
                !point.inFront ||
                point.depthMeters < 70.0 ||
                point.depthMeters > 2200.0
            ) {

                continue;
            }

            const float x =
                static_cast<float>(point.xPixels);

            const float y =
                static_cast<float>(point.yPixels);

            if (
                x < -120.0f ||
                x > width + 120.0f ||
                y < -100.0f ||
                y > horizonY - 12.0f
            ) {

                continue;
            }

            const float radius = std::clamp(
                static_cast<float>(
                    5000.0 / point.depthMeters
                ),
                8.0f,
                44.0f
            );

            drawCloud(
                window,
                x,
                y,
                radius
            );
        }
    }
}

// ============================================================
// 8. PRIMARY FLIGHT DISPLAY
// ============================================================

void Visuals13::drawPFD(
    sf::RenderWindow& window,
    const State& state,
    const ForcesMoments& loads
) const {

    // --------------------------------------------------------
    // A. PFD PANEL DIMENSIONS
    // --------------------------------------------------------

    constexpr float x = 16.0f;
    constexpr float y = 256.0f;

    constexpr float panelW = 280.0f;
    constexpr float panelH = 332.0f;

    constexpr float viewX = x + 12.0f;
    constexpr float viewY = y + 36.0f;

    constexpr float viewW = 256.0f;
    constexpr float viewH = 201.0f;

    const float cx =
        viewX + viewW / 2.0f;

    const float cy =
        viewY + viewH / 2.0f;

    const sf::Color white(239, 246, 255);
    const sf::Color amber(255, 197, 86);
    const sf::Color muted(170, 195, 213);

    // --------------------------------------------------------
    // B. DRAW PANEL BACKGROUND
    // --------------------------------------------------------

    sf::RectangleShape panel({
        panelW,
        panelH
    });

    panel.setPosition({x, y});

    panel.setFillColor(
        sf::Color(11, 24, 41, 244)
    );

    panel.setOutlineColor(
        sf::Color(92, 142, 177)
    );

    panel.setOutlineThickness(1.0f);

    window.draw(panel);

    drawText(
        window,
        "PRIMARY FLIGHT DISPLAY",
        15,
        {x + 12.0f, y + 7.0f},
        white
    );

    // --------------------------------------------------------
    // C. READ ACTUAL AIRCRAFT ORIENTATION
    // --------------------------------------------------------

    const EulerAngles attitude =
        toEuler(state.bodyToWorld);

    const double pitchDeg =
        degrees(attitude.pitchRadians);

    const double bankDeg =
        degrees(attitude.rollRadians);

    const float bank =
        static_cast<float>(
            attitude.rollRadians
        );

    // The aircraft reference symbol stays fixed.
    // The horizon moves with attitude.
    constexpr float pixelsPerDegree = 3.8f;

    const float offset =
        static_cast<float>(pitchDeg)
        * pixelsPerDegree;

    const Point normal{
        std::sin(bank),
        std::cos(bank)
    };

    const Point tangent{
        std::cos(bank),
        -std::sin(bank)
    };

    const Point horizon{
        cx + normal.x * offset,
        cy + normal.y * offset
    };

    const std::vector<Point> rectangle = {
        {viewX, viewY},
        {viewX + viewW, viewY},
        {viewX + viewW, viewY + viewH},
        {viewX, viewY + viewH}
    };

    // --------------------------------------------------------
    // D. MOVING SKY AND GROUND
    // --------------------------------------------------------

    drawPolygon(
        window,
        clipHorizon(
            rectangle,
            horizon,
            normal,
            true
        ),
        sf::Color(72, 145, 203)
    );

    drawPolygon(
        window,
        clipHorizon(
            rectangle,
            horizon,
            normal,
            false
        ),
        sf::Color(143, 95, 58)
    );

    // --------------------------------------------------------
    // E. WHITE HORIZON LINE
    // --------------------------------------------------------

    Point h1{
        horizon.x - tangent.x * 500.0f,
        horizon.y - tangent.y * 500.0f
    };

    Point h2{
        horizon.x + tangent.x * 500.0f,
        horizon.y + tangent.y * 500.0f
    };

    if (
        clipLine(
            h1,
            h2,
            viewX,
            viewY,
            viewX + viewW,
            viewY + viewH
        )
    ) {

        drawLine(
            window,
            h1,
            h2,
            white
        );
    }

    // --------------------------------------------------------
    // F. PITCH LADDER
    // --------------------------------------------------------

    for (
        int degree = -40;
        degree <= 40;
        degree += 10
    ) {

        if (degree == 0) {
            continue;
        }

        const float ladderOffset =
            static_cast<float>(
                pitchDeg - degree
            ) * pixelsPerDegree;

        const Point middle{
            cx + normal.x * ladderOffset,
            cy + normal.y * ladderOffset
        };

        Point a{
            middle.x - tangent.x * 28.0f,
            middle.y - tangent.y * 28.0f
        };

        Point b{
            middle.x + tangent.x * 28.0f,
            middle.y + tangent.y * 28.0f
        };

        if (
            clipLine(
                a,
                b,
                viewX + 2.0f,
                viewY + 2.0f,
                viewX + viewW - 2.0f,
                viewY + viewH - 2.0f
            )
        ) {

            drawLine(
                window,
                a,
                b,
                sf::Color(250, 250, 247, 205)
            );
        }
    }

    // --------------------------------------------------------
    // G. FIXED AIRCRAFT REFERENCE SYMBOL
    // --------------------------------------------------------

    drawLine(
        window,
        {cx - 55.0f, cy},
        {cx - 15.0f, cy},
        amber
    );

    drawLine(
        window,
        {cx + 15.0f, cy},
        {cx + 55.0f, cy},
        amber
    );

    drawLine(
        window,
        {cx - 15.0f, cy},
        {cx - 7.0f, cy + 8.0f},
        amber
    );

    drawLine(
        window,
        {cx + 15.0f, cy},
        {cx + 7.0f, cy + 8.0f},
        amber
    );

    // Fixed bank reference triangle.
    sf::ConvexShape bankPointer(3);

    bankPointer.setPoint(
        0,
        {cx, viewY + 4.0f}
    );

    bankPointer.setPoint(
        1,
        {cx - 6.0f, viewY + 16.0f}
    );

    bankPointer.setPoint(
        2,
        {cx + 6.0f, viewY + 16.0f}
    );

    bankPointer.setFillColor(amber);

    window.draw(bankPointer);

    // Draw the PFD viewport border.
    sf::RectangleShape border({
        viewW,
        viewH
    });

    border.setPosition({viewX, viewY});

    border.setFillColor(
        sf::Color::Transparent
    );

    border.setOutlineColor(
        sf::Color(212, 225, 237)
    );

    border.setOutlineThickness(1.0f);

    window.draw(border);

    // --------------------------------------------------------
    // H. READ FLIGHT INSTRUMENT DATA
    // --------------------------------------------------------

    const double altitudeFt =
        -state.positionWorldMeters.z
        / kFeetToMeters;

    const double trueAirspeedKt =
        loads.air.trueAirspeedMetersPerSecond
        / kKnotsToMetersPerSecond;

    const double verticalFtMin =
        -state.velocityWorldMetersPerSecond.z
        * 60.0 / kFeetToMeters;

    double heading =
        std::fmod(
            degrees(attitude.yawRadians),
            360.0
        );

    if (heading < 0.0) {
        heading += 360.0;
    }

    // --------------------------------------------------------
    // I. DRAW THE NUMERICAL FLIGHT DATA
    // --------------------------------------------------------

    // TAS = True Airspeed.
    // Our current physics model doesn't calculate IAS.
    drawText(
        window,
        "TAS " + number(trueAirspeedKt)
        + " kt   ALT " + number(altitudeFt)
        + " ft",
        14,
        {x + 12.0f, y + 247.0f},
        white
    );

    drawText(
        window,
        "VS " + number(verticalFtMin)
        + " ft/min  HDG " + number(heading),
        13,
        {x + 12.0f, y + 272.0f},
        white
    );

    drawText(
        window,
        "PITCH " + number(pitchDeg, 1)
        + "   BANK " + number(bankDeg, 1),
        12,
        {x + 12.0f, y + 301.0f},
        muted
    );
}

} // namespace flight::sixdof
