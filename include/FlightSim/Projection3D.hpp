
#pragma once

#include "FlightSim/Dynamics6DOF.hpp"

namespace flight::sixdof {

// ============================================================
// CAMERA
// ============================================================

// Camera position and orientation in world NED.
//
// 'upWorld' is the direction appearing upward
// on the screen, despite NED using positive Z downward.
struct Camera3D {

    Vec3 positionWorld{};

    Vec3 forwardWorld{1.0, 0.0, 0.0};

    Vec3 rightWorld{0.0, 1.0, 0.0};

    Vec3 upWorld{0.0, 0.0, -1.0};

    // Viewport dimensions in screen pixels.
    double viewportWidth = 1280.0;
    double viewportHeight = 720.0;

    // Vertical field of view.
    double verticalFovRadians = radians(65.0);

    // Hide geometry too close to or behind the camera.
    double nearPlaneMeters = 0.25;
};

// ============================================================
// PROJECTED SCREEN POINT
// ============================================================

struct ProjectedPoint {

    double xPixels = 0.0;
    double yPixels = 0.0;

    // Distance along the camera's forward direction.
    double depthMeters = 0.0;

    // Whether the point is in front of the near plane.
    bool inFront = false;
};

// Create a chase camera following the airplane.
//
// Camera follows heading but doesn't bank with it.
Camera3D makeChaseCamera(
    const State& state,
    double width,
    double height
);

// Convert a world-space 3D point to a 2D screen point.
ProjectedPoint projectPoint(
    const Camera3D& camera,
    Vec3 worldPoint
);

} // namespace flight::sixdof
