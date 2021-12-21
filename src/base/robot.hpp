#pragma once
#include <base/Math.hpp>
#include <vector>
#include <string>
#include <iostream>

#include "Eigen/Dense"

using namespace FW;

struct Vertex
{
	Vec3f position;
	Vec3f normal;
	Vec3f color;
};

// DH parameters for describing a link
struct DhParam {
	// a = translation along the x-axis of the previous frame
	// alpha = rotation around the x-axis of previous frame
	// d = offset from previous frame to current frame (translation along previous z)
	// theta = angle from previous frame to current frame (rotation along previous z)
	float d, a, alpha, theta;

	// type of joint: 'R' = revolute, 'P' = prismatic
	std::string sigma;
};

struct Link {
	// rotation angle of this link (or the rotation of the joint that moves this link)
	float rotation;

	// target rotation for this link, current rotation will move towards this when update() is called
	float target_rotation;

	// when true, the rotation cannot be changed
	bool is_locked;

	float joint_speed;
	bool is_moving;

	// DH parameters of this link
	DhParam p;

	// components of the link transformation matrix
	// calculated from the DH parameters of this link
	FW::Mat4f z_screw;
	FW::Mat4f x_screw;

	// combination of applying z_screw and  then x_screw
	// relates this link's frame to the frame of the previous link
	FW::Mat4f link_matrix;

	// used to evaluate the link matrix for any joint angle state
	// does not modify current state of the struct
	// used by iterative inverse kinematics solvers
	FW::Mat4f eval_link_matrix(float rotationAngle) const;

	// transforms the coordinates of this link frame to world frame
	// created by multiplying together all link matrices of the previous links and the current link
	FW::Mat4f to_world;
};

class Robot {
	 
public:
	Robot(std::string dh_param_filename, Vec3f location);

	// used to update robot angles over some time difference dt
	void					update(float dt);

	// kinematics stuff
	FW::Vec3f				getTcpWorldPosition() const;
	FW::Vec3f				getTcpWorldPosition(Eigen::VectorXf jointAngles) const; // overload used by iterative inverse kinematics solutions
	Eigen::VectorXf			getTcpSpeed() const;
	Eigen::VectorXf			inverseKinematics(FW::Vec3f tcp_pos) const;
	Eigen::MatrixXf			getJacobian() const;
	Eigen::MatrixXf			getJacobian(Eigen::VectorXf jointAngles) const;	// overload used by iterative inverse kinematics solutions
	Eigen::VectorXf			getJointSpeeds() const;

	// joint angle getters
	Eigen::VectorXf			getJointAngles() const;
	float					getJointAngle(unsigned index) const { return links_[index].rotation; };
	float					getSelectedJointAngle() const { return links_[selected_joint_].rotation; };
	Eigen::VectorXf			getTargetJointAngles() const;
	float					getTargetJointAngle(unsigned index) const { return links_[index].target_rotation; };
	float					getSelectedJointTargetAngle() const { return links_[selected_joint_].target_rotation; };
	unsigned				getSelectedJoint() const { return selected_joint_; };

	// joint angle setters
	void					setJointTargetAngle(unsigned index, float angle);
	void					setSelectedJoint(unsigned index) { selected_joint_ = index; };

	// getters used by graphics interface
	std::vector<FW::Mat4f>		getToWorldTransforms() const;
	const std::vector<Link>&	getLinks() const;

	size_t					getNumJoints() const { return links_.size() - 1;  /* zeroth link is not counted */ };

private:
	Mat4f worldToBase_;
	Mat4f baseToZero_;

	void					updateToWorldTransforms();
	std::vector<DhParam>	loadDhParams(const std::string filename);
	void					buildKinematicModel(const std::vector<DhParam>& params);

	// private setters for changing joint rotations directly (must call updateToWorldTransforms)
	void					setJointAngle(unsigned index, float angle);
	void					incrJointAngle(unsigned index, float angle);

	const float JOINT_POSITIONAL_ACCURACY = 0.001f;
	const float IK_POS_THRESHOLD = 0.0001f;

	std::vector<Link> links_;
	unsigned selected_joint_;
};

