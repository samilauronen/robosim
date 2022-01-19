#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>

#include "Camera.hpp"

using namespace Eigen;

Camera::Camera(
    const Eigen::Vector3f& position,
    const Eigen::Vector3f& forward,
    const Eigen::Vector3f& up, float speed, float mouse_sensitivity
):
    position_(position),
    forward_(forward),
    up_(up),
    speed_(speed),
    mouse_sensitivity_(mouse_sensitivity),
    fov_(70.0f),
    near_(0.001f),
    far_(3.0f),
    dragLeft_(false),
    dragMiddle_(false),
    dragRight_(false)
{
}

void Camera::update(float dt, GLFWwindow* window)
{
    Matrix3f orient = getOrientation();
    Vector3f rotate = Vector3f::Zero();
    Vector3f move = Vector3f::Zero();

    // check if mouse is dragged with a button pressed
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS)   dragLeft_ = true;
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS)   dragMiddle_ = true;
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS)    dragRight_ = true;
    // check if released
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_RELEASE)   dragLeft_ = false;
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_RELEASE) dragMiddle_ = false;
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_RELEASE) dragRight_ = false;

    // calculate mouse delta
    double mouse_x, mouse_y;
    glfwGetCursorPos(window, &mouse_x, &mouse_y);
    Vector3f delta = Vector3f(mouse_x - last_mouse_pos.x(), last_mouse_pos.y() - mouse_y, 0.0f);
    last_mouse_pos.x() = mouse_x;
    last_mouse_pos.y() = mouse_y;

    // increment rotation
    if (dragMiddle_) rotate += delta * mouse_sensitivity_;

    // check movement keys and set movement vector
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) move.x() -= 1.0f;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) move.x() += 1.0f;
    if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS) move.y() -= 1.0f;
    if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS) move.y() += 1.0f;
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) move.z() -= 1.0f;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) move.z() += 1.0f;
    move *= dt * speed_;

    // apply movement and rotation
    if (!move.isZero()) {
        position_ += orient * move;
    }
    if (rotate.x() != 0.0f || rotate.y() != 0.0f)
    {
        Vector3f tmp = orient.col(2) * cos(rotate.x()) - orient.col(0) * sin(rotate.x());
        forward_ = (orient.col(1) * sin(rotate.y()) - tmp * cos(rotate.y())).normalized();
        up_ = (orient.col(1) * cos(rotate.y()) + tmp * sin(rotate.y())).normalized();
    }
    if (rotate.z() != 0.0f)
    {
        Vector3f up = orient.transpose() * up_;
        up_ = orient * Vector3f(up.x() * cos(rotate.z()) - sin(rotate.z()), up.x() * sin(rotate.z()) + up.y() * cos(rotate.z()), up.z());
    }
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
