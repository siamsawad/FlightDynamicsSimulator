
#include "FlightSim/Projection3D.hpp"

#include <cmath>
#include <stdexcept>

namespace flight::sixdof {

namespace {

// Normalize a vector to unit length.
Vec3 unit(Vec3 v) {

    const double length = magnitude(v);

    if (
        !std::isfinite(length)
        || length < 1e-10
    ) {

        throw std::invalid_argument(
            "Cannot normalize a zero-length vector"
        );
    }

    return v / length;
}

} // anonymous namespace

// ============================================================
// CREATE CHASE CAMERA
// ============================================================

Camera3D makeChaseCamera(
    const State& state,
    double width,
    double height
) {

    // Validate viewport dimensions.
    if (
        !std::isfinite(width)
        || !std::isfinite(height)
        || width <= 0.0
        || height <= 0.0
    ) {

        throw std::invalid_argument(
            "Invalid camera viewport"
        );
    }

    // Determine the aircraft's forward direction
    // using its actual 6-DOF quaternion orientation.
    const Vec3 bodyForward = rotateBodyToWorld(
        state.bodyToWorld,
        {1.0, 0.0, 0.0}
    );

    // Use horizontal heading so the chase camera
    // doesn't pitch or roll directly with the airplane.
    Vec3 heading{
        bodyForward.x,
        bodyForward.y,
        0.0
    };

    // Safe fallback for nearly vertical aircraft.
    if (magnitude(heading) < 1e-6) {
        heading = {1.0, 0.0, 0.0};
    }

    heading = unit(heading);

    Camera3D camera;

    camera.viewportWidth = width;
    camera.viewportHeight = height;

    // Position behind and above the aircraft.
    //
    // NED convention:
    // Negative Z means upward.
    camera.positionWorld =
        state.positionWorldMeters
        - heading * 27.0
        + Vec3{0.0, 0.0, -10.0};

    // Look slightly ahead of the airplane.
    const Vec3 lookAt =
        state.positionWorldMeters
        + heading * 5.0;

    camera.forwardWorld = unit(
        lookAt - camera.positionWorld
    );

    // The world-down axis is positive Z.
    //
    // Cross products create the camera's
    // perpendicular right and up directions.
    camera.rightWorld = unit(
        cross(
            {0.0, 0.0, 1.0},
            camera.forwardWorld
        )
    );

    camera.upWorld = unit(
        cross(
            camera.rightWorld,
            camera.forwardWorld
        )
    );

    return camera;
}

// ============================================================
// PROJECT A WORLD POINT ONTO THE SCREEN
// ============================================================

ProjectedPoint projectPoint(
    const Camera3D& camera,
    Vec3 worldPoint
) {

    // Vector from camera to the world-space point.
    const Vec3 relative =
        worldPoint - camera.positionWorld;

    // Depth along the camera's forward axis.
    const double depth = dot(
        relative,
        camera.forwardWorld
    );

    ProjectedPoint projected;
    projected.depthMeters = depth;

    // Hide points behind or extremely close to camera.
    if (
        !std::isfinite(depth)
        || depth <= camera.nearPlaneMeters
    ) {

        return projected;
    }

    // Perspective focal length, measured in pixels.
    const double focalPixels =
        (camera.viewportHeight * 0.5) /
        std::tan(
            camera.verticalFovRadians * 0.5
        );

    // Project horizontal coordinate.
    projected.xPixels =
        camera.viewportWidth * 0.5
        + focalPixels
        * dot(relative, camera.rightWorld)
        / depth;

    // Project vertical coordinate.
    // Negative sign because screen Y increases downward.
    projected.yPixels =
        camera.viewportHeight * 0.5
        - focalPixels
        * dot(relative, camera.upWorld)
        / depth;

    projected.inFront =
        std::isfinite(projected.xPixels)
        && std::isfinite(projected.yPixels);

    return projected;
}

} // namespace flight::sixdof
