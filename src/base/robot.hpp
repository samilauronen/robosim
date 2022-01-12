#pragma once
#include <base/Math.hpp>
#include <vector>
#include <string>
#include <iostream>

#include "DhParam.hpp"
#include "JointedLink.hpp"
#include "Eigen/Dense"

class Robot {
public:
	Robot(std::string dh_param_filename, FW::Vec3f location);

	// used to update joint angles over some time difference dt
	void					update(float dt);

	// kinematics stuff
	FW::Vec3f				getTcpWorldPosition() const;
	FW::Vec3f				getTcpWorldPosition(Eigen::VectorXf jointAngles) const;
	Eigen::VectorXf			getTcpSpeed() const;
	Eigen::MatrixXf			getJacobian() const;
	Eigen::MatrixXf			getJacobian(Eigen::VectorXf jointAngles) const;
	Eigen::VectorXf			getJointSpeeds() const;

	// ik targets are always wrt world frame
	FW::Vec3f				getTargetTcpPosition() const { return ik_target_; };
	void					setTargetTcpPosition(FW::Vec3f new_target_wrt_world);
	Eigen::VectorXf			solveInverseKinematics(FW::Vec3f tcp_pos) const;

	// joint angle getters
	Eigen::VectorXf			getJointAngles() const;
	float					getJointAngle(unsigned index) const { return links_[index].getJointRotation(); };
	Eigen::VectorXf			getTargetJointAngles() const;
	float					getTargetJointAngle(unsigned index) const { return links_[index].getJointTargetRotation(); };

	// joint angle setters
	void					setJointTargetAngles(Eigen::VectorXf angles);
	void					setJointTargetAngle(unsigned index, float angle);

	void					renderSkeleton() const;
	std::vector<Vertex>		getMeshVertices() const;

	size_t					getNumJoints() const { return links_.size(); };
	FW::Mat4f				getWorldToBase() const { return worldToBase_; };

private:
	FW::Mat4f worldToBase_;

	void					createLinks(const std::vector<DhParam>& params);

	const float JOINT_POSITIONAL_ACCURACY = 0.001f;

	std::vector<JointedLink> links_;
	FW::Vec3f ik_target_;
};

