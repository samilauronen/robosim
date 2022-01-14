#pragma once

#include <vector>
#include <memory>

#include "Eigen/Dense"
#include "DhParam.hpp"
#include "meshes/JointedLinkMesh.hpp"

// GLEW
#define GLEW_STATIC
#include <GL/glew.h>

// GLFW
#include <GLFW/glfw3.h>

// describes a combination of a joint and a link
class JointedLink {
public:
	JointedLink(DhParam params, float joint_rotation, int link_number);

	// updates rotation based on current joint speed and time passed since last update
	// also updates the link matrix and mesh position
	void update(float dt, Eigen::Affine3f current_world_transform);

	// used to evaluate the link matrix for any joint angle state
	// useful for iterative inverse kinematics solvers
	Eigen::Affine3f evalLinkMatrix(float rotationAngle) const;

	// setters
	void setToWorld(const Eigen::Affine3f& to_world) { to_world_ = to_world; };
	void setJointRotation(float rotation_angle) { rotation_ = rotation_angle; };
	void setJointSpeed(float new_speed) { joint_speed_ = new_speed; };
	void setJointTargetRotation(float target_angle) { target_rotation_ = target_angle; };

	// matching getters
	Eigen::Affine3f getToWorld() const { return to_world_; };
	float getJointRotation() const { return rotation_; };
	float getJointSpeed() const { return joint_speed_; };
	float getJointTargetRotation() const { return target_rotation_; };

	// no setter
	DhParam getDhParams() const { return params_; };
	Eigen::Affine3f getLinkMatrix() const { return link_matrix_; };

	// returns the vertices of the mesh that represents the combination of a joint and a link
	std::vector<Vertex> getMeshVertices() const;
	void renderSkeleton() const;

private:
	void updateLinkMatrix(float rotationAngle);
	void updateMesh();

	DhParam params_;		// DH parameters of this link

	int link_number_;		// the ordinal number of this link in the robot arm's link chain, starting from base

	JointedLinkMesh mesh_;		// mesh used for graphical representation

	float joint_speed_;     // current speed of the joint in rad/s
	float rotation_;		// rotation angle of the joint moving this link
	float target_rotation_;	// target rotation for the joint, current rotation will move towards this when update() is called

	// combination of applying z_screw and then x_screw
	// relates this link's frame to the frame of the previous link
	Eigen::Affine3f link_matrix_;

	// transforms the coordinates of this link frame to world frame
	// created by multiplying together all link matrices of the previous links and the current link
	// needs to be set by the Robot class
	Eigen::Affine3f to_world_;
};