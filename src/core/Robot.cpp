#include <algorithm>
#include <assert.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include "Robot.hpp"
#include "Utility.hpp"
#include "InverseKinematics.hpp"

using namespace Eigen;
using namespace std;

// TODO:
/*
* Unify naming conventions, camelCase vs snake_case
* Do something with constants, maybe separate file? At least place them so that they make sense
* Think about the naming convention of matrices: to vs from as left or right multiply
* Make the plane be drawn using shaders for more efficiency
* Create a Scene class to have lights and stuff, then update it with dt to have the light move with const speed
* Simplify jacobian calculation
* Use ImGui for UI
* Make prismatic joints possible?
* Maybe have a toggle for drawing a wireframe-ball around the robot, visualizing it's range
* inverse jacobians!, for trajectories!
* Switch to quternions for trajectories to really work (interpolating orientation)
* Make joints have an acceleration profile like in the lecture slides
* Dynamics???
*/

/*
* Note:
* Robot frames are assigned with zero-based indexing, so that the first joint has index 0
* and it moves link 0, at the end of which is frame 0.
* The first frame, which is conventionally the zero:th frame, is now simply the base frame
* This means that the worldToBase transform describes what conventionally would be worldToBase * baseToZero
*/
Robot::Robot(std::string dh_param_filename, Vector3f location)
{
	// set the transformation from world to base, which includes rotation to turn z-axis upwards
	// and the robots location in the world
	worldToBase_ = AngleAxisf(-M_PI / 2, Vector3f::UnitX()) * Translation3f(location);

	// create kinematic model and links of the robot
	vector<DhParam> params = loadDhParamsFromFile(dh_param_filename);
	this->createLinks(params);
}

void Robot::createLinks(const std::vector<DhParam>& dh_params)
{
	int link_index = 0;
	for (const DhParam& link_params : dh_params) {
		cout << "Parameters for link " << link_index << ":" << endl;
		cout << "	length: " << link_params.a << endl;
		cout << "	twist: " << link_params.alpha << endl;
		cout << "	offset: " << link_params.d << endl;
		cout << "	rotation: " << 0 << endl;

		JointedLink link(link_params, 0.0f, link_index);
		links_.push_back(link);
		link_index++;
	}
	cout << "Created " << links_.size() << " links" << endl << endl;
	cout << "Manipulator Jacobian when joint angles are zero: " << endl;
	cout << getJacobian() << endl;
}

void Robot::update(float dt)
{
	// in rad/s
	const float JOINT_SPEED = M_PI / 3;

	// chain the link transformations like so:
	// for link 0 the result should be W_T_0 (which is W_T_B * B_T_0)
	// for link 1: W_T_1
	// for link 2: W_T_2
	// etc...
	// index always tells which frame is the result of the transformation
	Affine3f current_to_world = worldToBase_;

	for (JointedLink& link : links_) {
		float target = link.getJointTargetRotation();
		float current = link.getJointRotation();

		float diff = current - target;

		const float JOINT_POSITIONAL_ACCURACY = 0.001f;
		// determintes whether joint should move
		// TODO: create a joint controller class for doing this?
		// or maybe move this inside the JointedLink class
		if (abs(diff) > JOINT_POSITIONAL_ACCURACY) {
			link.setJointSpeed(current < target ? JOINT_SPEED : -JOINT_SPEED);
		} else {
			link.setJointSpeed(0);
		}

		link.update(dt, current_to_world);
		current_to_world = current_to_world * link.getLinkMatrix();
	}
}

// TCP position for current joint angles
Vector3f Robot::getTcpWorldPosition() const
{
	return links_[links_.size() - 1].getToWorld().translation();
}

Vector3f Robot::getTcpWorldPosition(VectorXf jointAngles) const
{
	Affine3f to_world = worldToBase_;
	for (int i = 0; i < links_.size(); i++) {
		to_world = to_world * links_[i].evalLinkMatrix(jointAngles(i));
	}
	return to_world.translation();
}

VectorXf Robot::getTcpSpeed() const
{
	return getJacobian() * getJointSpeeds();
}

void Robot::setTargetTcpPosition(Vector3f new_target_wrt_world)
{
	ik_target_ = new_target_wrt_world;
	VectorXf targetAngles = solveInverseKinematics(ik_target_);
	setJointTargetAngles(targetAngles);
}

VectorXf Robot::solveInverseKinematics(Vector3f target_tcp_world_pos) const
{
	// solver can be switched quickly
	IK::IKSolution solution = IK::SimpleIKSolver::solve(*this, target_tcp_world_pos);

	// if target was not reached, return current joint angles
	if (solution.timed_out) {
		cout << "IK solution timed out after " << solution.time_taken_micros << " microseconds" << endl;
		return getJointAngles();
	}

	// solution was reached
	cout << "==============" << endl;
	cout << "IK solution received in " << solution.time_taken_micros << " microseconds" << endl;
	cout << solution.joint_angles << endl;
	cout << "==============" << endl;
	return solution.joint_angles;
}

MatrixXf Robot::getJacobian(VectorXf jointAngles) const {
	int n_rows = 6;					// jacobian will always have 6 rows, since 3D space has 6 degrees of freedom
	int n_cols = getNumJoints();	// number of columns is the number of joint angles that can be controlled

	MatrixXf jacobian(n_rows, n_cols);

	// returns the transform B_T_i as Matrix4f
	// returns B_T_B if i = -1
	auto get_base_to_i = [&](int i) {
		Affine3f res = Affine3f::Identity();
		for (int j = 0; j <= i; j++) {
			res = res * links_[j].evalLinkMatrix(jointAngles(j));
		}
		return res;
	};

	// https://automaticaddison.com/the-ultimate-guide-to-jacobian-matrices-for-robotics/
	Affine3f base_to_n = get_base_to_i(links_.size() - 1);
	Vector3f transl_base_to_n = base_to_n.translation();

	// transformations from frame zero to frame i
	Affine3f tf_b_to_i;
	Matrix3f rot_b_to_i;
	Vector3f transl_b_to_i;

	Vector3f z_vec = Vector3f(0, 0, 1);

	// fill the matrix column-by-column
	int numJoints = (int)links_.size();
	for (int i = -1; i < numJoints - 1; i++) {
		tf_b_to_i = get_base_to_i(i);
		rot_b_to_i = tf_b_to_i.rotation();
		transl_b_to_i = tf_b_to_i.translation();

		Vector3f upper_part = (rot_b_to_i * z_vec).cross(transl_base_to_n - transl_b_to_i);
		Vector3f lower_part = rot_b_to_i * z_vec;

		Vector<float, 6> col;
		col << upper_part, lower_part;
		jacobian.col(i + 1) = col;
	}

	return jacobian;
}

// jacobian for current joint angles
MatrixXf Robot::getJacobian() const
{
	return getJacobian(getJointAngles());
}

VectorXf Robot::getJointSpeeds() const
{
	VectorXf speeds(getNumJoints());

	for (int i = 0; i < links_.size(); i++) {
		speeds(i) = links_[i].getJointSpeed();
	}
	return speeds;
}

VectorXf Robot::getJointAngles() const
{
	VectorXf angles(getNumJoints());
	for (int i = 0; i < links_.size(); i++) {
		angles(i) = links_[i].getJointRotation();
	}
	return angles;
}

VectorXf Robot::getTargetJointAngles() const
{
	VectorXf angles(getNumJoints());
	for (int i = 0; i < links_.size(); i++) {
		angles(i) = links_[i].getJointTargetRotation();
	}
	return angles;
}
void Robot::setJointTargetAngles(VectorXf angles) {
	assert(angles.rows() == getNumJoints());
	for (int i = 0; i < angles.rows(); i++) {
		links_[i].setJointTargetRotation(angles(i));
	}
}

void Robot::setJointTargetAngle(unsigned index, float angle)
{
	links_[index].setJointTargetRotation(angle);
}

void Robot::renderSkeleton() const
{
	for (const JointedLink& link : links_) {
		link.renderSkeleton();
	}
}

std::vector<Vertex> Robot::getMeshVertices() const
{
	std::vector<Vertex> all_vertices;
	for (int i = 0; i < links_.size(); i++) {
		std::vector<Vertex> link_vertices = links_[i].getMeshVertices();
		all_vertices.insert(all_vertices.end(), link_vertices.begin(), link_vertices.end());
	}
	return all_vertices;
}
