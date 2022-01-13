#pragma once

#include "Eigen/Dense"

class Camera
{
public:
	Camera(
		const Eigen::Vector3f& position,
		const Eigen::Vector3f& forward,
		const Eigen::Vector3f& up,
		float speed = 100.0f, float mouse_sensitivity = 0.01f
	);

	void setSpeed(float new_speed) { speed_ = new_speed; };
	void setMouseSensitivity(float new_sensitivity) { mouse_sensitivity_ = new_sensitivity; };

	Eigen::Vector3f getUp() const { return up_; };
	Eigen::Vector3f getForward() const { return forward_; };
	Eigen::Matrix4f getWorldToCamera(void) const;
	Eigen::Matrix4f getCameraToClip() const;
	Eigen::Matrix4f getWorldToClip() const { return getCameraToClip() * getWorldToCamera(); }

	Eigen::Matrix3f getOrientation() const;

	void handleEvent(const float dummy_event, float dt);

private:
	Eigen::Vector3f position_;
	Eigen::Vector3f forward_;
	Eigen::Vector3f up_;

	bool dragLeft_, dragMiddle_, dragRight_;

	float fov_, near_, far_;

	float speed_;
	float mouse_sensitivity_;
};