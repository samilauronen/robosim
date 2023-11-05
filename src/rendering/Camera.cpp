#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>

#include "Camera.hpp"

using namespace Eigen;

Camera::Camera(
    const Eigen::Vector3f& center, const float radius,
    const float azimuth, const float polar,
    const Eigen::Vector3f& up, float speed, float mouse_sensitivity
):
    center_(center),
    azimuthAngle_(azimuth),
    polarAngle_(polar),
    speed_(speed),
    up_(up),
    mouse_sensitivity_(mouse_sensitivity),
    fov_(70.0f),
    near_(0.001f),
    far_(3.0f),
    dragLeft_(false),
    dragMiddle_(false),
    dragRight_(false),
    shiftDown_(false),
    radius_(radius)
{
    // Calculate sines / cosines of angles
    const auto sineAzimuth = sin(azimuthAngle_);
    const auto cosineAzimuth = cos(azimuthAngle_);
    const auto sinePolar = sin(polarAngle_);
    const auto cosinePolar = cos(polarAngle_);

    // Calculate eye position out of them
    const auto x = center_.x() + radius_ * cosinePolar * cosineAzimuth;
    const auto y = center_.y() + radius_ * sinePolar;
    const auto z = center_.z() + radius_ * cosinePolar * sineAzimuth;

    position_ = {x,y,z};

    // calculate forward
    forward_ = (center_ - position_).normalized();
}

void Camera::update(float dt, GLFWwindow* window)
{
    // check if mouse is dragged with a button pressed
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS)   dragLeft_ = true;
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS)   dragMiddle_ = true;
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS)    dragRight_ = true;
    // check if released
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_RELEASE)   dragLeft_ = false;
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_RELEASE) dragMiddle_ = false;
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_RELEASE) dragRight_ = false;

    // check keyboard button presses and releases
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)    shiftDown_ = true;
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_RELEASE)  shiftDown_ = false;



    // calculate mouse delta
    double mouse_x, mouse_y;
    glfwGetCursorPos(window, &mouse_x, &mouse_y);
    Vector3f delta = Vector3f(mouse_x - last_mouse_pos.x(), last_mouse_pos.y() - mouse_y, 0.0f);
    last_mouse_pos.x() = mouse_x;
    last_mouse_pos.y() = mouse_y;

    // movement
    if (dragMiddle_)
    {
        if (shiftDown_)
        {
            // move center of rotation
        }
        else
        {
            // rotate azimuth by mouse delta in that direction
            azimuthAngle_ += mouse_sensitivity_ * delta.x();

            // Keep azimuth angle within range <0..2PI) - it's not necessary, just to have it nicely output
            const auto fullCircle = 2.0f * M_PI;
            azimuthAngle_ = fmodf(azimuthAngle_, fullCircle);
            if (azimuthAngle_ < 0.0f)
            {
                azimuthAngle_ = fullCircle + azimuthAngle_;
            }

            polarAngle_ -= mouse_sensitivity_ * delta.y();

            // Check if the angle hasn't exceeded quarter of a circle to prevent flip, add a bit of epsilon like 0.001 radians
            const auto polarCap = M_PI / 2.0f - 0.001f;
            if (polarAngle_ > polarCap) {
                polarAngle_ = polarCap;
            }

            if (polarAngle_ < -polarCap) {
                polarAngle_ = -polarCap;
            }

        }
    }

    // check movement keys and set movement vector
    // if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) move.x() -= 1.0f;
    // if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) move.x() += 1.0f;
    // if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS) move.y() -= 1.0f;
    // if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS) move.y() += 1.0f;
    // if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) move.z() -= 1.0f;
    // if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) move.z() += 1.0f;

    // Calculate sines / cosines of angles
    const auto sineAzimuth = sin(azimuthAngle_);
    const auto cosineAzimuth = cos(azimuthAngle_);
    const auto sinePolar = sin(polarAngle_);
    const auto cosinePolar = cos(polarAngle_);

    // Calculate eye position out of them
    const auto x = center_.x() + radius_ * cosinePolar * cosineAzimuth;
    const auto y = center_.y() + radius_ * sinePolar;
    const auto z = center_.z() + radius_ * cosinePolar * sineAzimuth;

    // apply the new position
    position_ = Vector3f(x,y,z);

    // apply new orientation
    forward_ = (center_ - position_).normalized();
    Vector3f right = forward_.cross(Vector3f(0,1,0));
    up_ = right.cross(forward_);
}

Eigen::Matrix4f Camera::getWorldToCamera(void) const
{
    Matrix3f orient = getOrientation();
    Vector3f pos = orient.transpose() * position_;
    Affine3f r;
    r.linear() = orient.transpose();
    r.translation() = -pos;
    return r.matrix();
}

Eigen::Matrix4f Camera::getCameraToClip() const
{
    // Camera points towards -z.  0 < near < far.
    // Matrix maps z range [-near, -far] to [-1, 1], after homogeneous division.
    float f = tan(fov_ * M_PI / 360.0f);
    float d = near_ - far_;
    f = f ? 1 / f : 0;
    d = d ? 1 / d : 0;

    Matrix4f r;
    r.row(0) = Vector4f(f, 0.0f, 0.0f, 0.0f);
    r.row(1) = Vector4f(0.0f, f, 0.0f, 0.0f);
    r.row(2) = Vector4f(0.0f, 0.0f, (near_ + far_) * d, 2.0f * near_ * far_ * d);
    r.row(3) = Vector4f(0.0f, 0.0f, -1.0f, 0.0f);
    return r;
    return Matrix4f{};
}


Eigen::Matrix3f Camera::getOrientation() const
{
    Eigen::Matrix3f r;
    r.col(2) = -forward_.normalized();
    r.col(0) = up_.cross(r.col(2)).normalized();
    r.col(1) = (r.col(2)).cross(r.col(0)).normalized();
    return r;
}
