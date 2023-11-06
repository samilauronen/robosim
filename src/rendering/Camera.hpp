#pragma once

#include "GLFW/glfw3.h"
#include "Eigen/Dense"

#include "core/Event.hpp"

class Camera
{
public:
	Camera(
		const Eigen::Vector3f& center, const float radius,
		const float azimuth, const float polar,
		const Eigen::Vector3f& up = Eigen::Vector3f(0.0f, 1.0f, 0.0f),
		float speed = 6.0f, float mouse_sensitivity = 0.01f
	);

	void setSpeed(float new_speed) { speed_ = new_speed; };
	void setMouseSensitivity(float new_sensitivity) { mouse_sensitivity_ = new_sensitivity; };

	float			getFOV() const { return fov_; };
	Eigen::Vector3f getUp() const { return up_; };
	Eigen::Vector3f getForward() const { return forward_; };
	Eigen::Vector3f getPosition() const { return position_; };
	Eigen::Matrix4f getWorldToCamera() const;
	Eigen::Matrix4f getCameraToClip() const;
	Eigen::Matrix4f getWorldToClip() const { return getCameraToClip() * getWorldToCamera(); }

	Eigen::Matrix3f getOrientation() const;

	void update(float dt, GLFWwindow* window);

	void handleEvent(const Event& ev);

private:
	Eigen::Vector3f position_;	// Position of the camera
	Eigen::Vector3f center_;	// Center of the orbit camera sphere (the point upon which the camera looks)
	Eigen::Vector3f forward_;
	Eigen::Vector3f up_;

    float radius_; 			// Radius of the orbit camera sphere
    float _minRadius;	 	// Minimal radius of the orbit camera sphere (cannot fall below this value)
    float azimuthAngle_; 	// Azimuth angle on the orbit camera sphere
    float polarAngle_;	 	// Polar angle on the orbit camera sphere

	bool dragLeft_, dragMiddle_, dragRight_, shiftDown_;

	float fov_, near_, far_;

	Eigen::Vector2d last_mouse_pos;

	float speed_;
	float mouse_sensitivity_;
};