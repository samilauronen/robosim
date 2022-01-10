#include "Robot.hpp"
#include "Utility.hpp"
#include "InverseKinematics.hpp"

#include <algorithm>
#include <assert.h>

using namespace FW;
using namespace std;

// TODO:
/*
* Don't create meshes again for every frame, that just sucks! Make RobotGraphics more oop style
* Convert all maths to using eigen
* Unify naming conventions, camelCase vs snake_case
* Use GLU for the meshes (should be included with OpenGL, if not, download it. It's different from GLUT which is not recommended anymore)
* Use GLFW or some other window manager
* Think about the naming convention of matrices: to vs from as left or right multiply
* Use ImGui for UI
* Make your own camera class, or use an existing one (from GLFW?)
* Make prismatic joints possible?
* Maybe have a toggle for drawing a wireframe-ball around the robot, visualizing it's range
* inverse jacobians!, for trajectories!
* Switch to quternions for trajectories to really work (interpolating orientation)
* Make joints have an acceleration profile like in the lecture slides
* Dynamics???
* ImGui for more awesome ui?
*/

/*
* Note:
* Robot frames are assigned with zero-based indexing, so that the first joint has index 0
* and it moves link 0, at the end of which is frame 0.
* The first frame, which is conventionally the zero:th frame, is now simply the base frame
* This means that the worldToBase transform describes what conventionally would be worldToBase * baseToZero
*/
Robot::Robot(std::string dh_param_filename, FW::Vec3f location)
{
	// set the transformation from world to base, whicj includes rotation to turn z-axis upwards
	// and the robots location in the world
	worldToBase_ = combineToMat4f(Mat3f::rotation(Vec3f(1, 0, 0), -FW_PI / 2), location);

	// create kinematic model and links of the robot
	vector<DhParam> params = loadDhParamsFromFile(dh_param_filename);
	buildKinematicModel(params);
}

void Robot::update(float dt_millis)
{
	// in rad/s
	const float JOINT_SPEED = FW_PI / 3;

	for (int i = 0; i < links_.size(); i++) {
		float target = getTargetJointAngle(i);
		float current = getJointAngle(i);

		float diff = current - target;

		if (FW::abs(diff) > JOINT_POSITIONAL_ACCURACY) {
			links_[i].setJointSpeed(current < target ? JOINT_SPEED : -JOINT_SPEED);
			// maybe call joint.update here?
			links_[i].setJointRotation(links_[i].getJointRotation() + links_[i].getJointSpeed() * (dt_millis / 1000));
		}
		else {
			links_[i].setJointSpeed(0);
		}
	}
	updateToWorldTransforms();
}

FW::Vec3f Robot::getTcpWorldPosition() const
{
	return links_[links_.size() - 1].getToWorld() * Vec3f(0, 0, 0);
}

FW::Vec3f Robot::getTcpWorldPosition(Eigen::VectorXf jointAngles) const
{
	Mat4f to_world = worldToBase_;
	for (int i = 0; i < links_.size(); i++) {
		to_world *= links_[i].evalLinkMatrix(jointAngles(i));
	}
	return to_world * Vec3f(0, 0, 0);
}

Eigen::VectorXf Robot::getTcpSpeed() const
{
	Eigen::MatrixXf jacobian = getJacobian();
	Eigen::VectorXf joint_speeds = getJointSpeeds();
	return jacobian * joint_speeds;
}

void Robot::setTargetTcpPosition(FW::Vec3f new_target_wrt_world)
{
	ik_target_ = new_target_wrt_world;
	Eigen::VectorXf targetAngles = solveInverseKinematics(ik_target_);
	setJointTargetAngles(targetAngles);
}

Eigen::VectorXf Robot::solveInverseKinematics(Vec3f target_tcp_world_pos) const
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

Eigen::MatrixXf Robot::getJacobian(Eigen::VectorXf jointAngles) const {
	int n_rows = 6;					// jacobian will always have 6 rows, since 3D space has 6 degrees of freedom
	int n_cols = getNumJoints();	// number of columns is the number of joint angles that can be controlled

	Eigen::MatrixXf jacobian(n_rows, n_cols);

	// returns the transform B_T_i as Eigen::Matrix4f
	// returns B_T_B if i = -1
	auto get_base_to_i = [&](int i) {
		Mat4f fwres;
		for (int j = 0; j <= i; j++) {
			fwres *= links_[j].evalLinkMatrix(jointAngles(j));
		}

		Eigen::Matrix4f res;
		Vec4f row0 = fwres.getRow(0);
		Vec4f row1 = fwres.getRow(1);
		Vec4f row2 = fwres.getRow(2);
		Vec4f row3 = fwres.getRow(3);

		res << row0.x, row0.y, row0.z, row0.w,
			row1.x, row1.y, row1.z, row1.w,
			row2.x, row2.y, row2.z, row2.w,
			row3.x, row3.y, row3.z, row3.w;
		return res;
	};

	// https://automaticaddison.com/the-ultimate-guide-to-jacobian-matrices-for-robotics/
	Eigen::Matrix4f base_to_n = get_base_to_i(links_.size() - 1);
	Eigen::Vector3f transl_base_to_n = base_to_n.col(3).head<3>();

	// transformations from frame zero to frame i
	Eigen::Matrix4f tf_b_to_i;
	Eigen::Matrix3f rot_b_to_i;
	Eigen::Vector3f transl_b_to_i;

	Eigen::Vector3f z_vec = Eigen::Vector3f(0, 0, 1);

	// fill the matrix column-by-column
	int numJoints = (int)links_.size();
	for (int i = -1; i < numJoints - 1; i++) {
		tf_b_to_i = get_base_to_i(i);
		rot_b_to_i = tf_b_to_i.block<3, 3>(0, 0);
		transl_b_to_i = tf_b_to_i.block<3, 1>(0, 3);

		Eigen::Vector3f upper_part = (rot_b_to_i * z_vec).cross(transl_base_to_n - transl_b_to_i);
		Eigen::Vector3f lower_part = rot_b_to_i * z_vec;

		Eigen::Vector<float, 6> col;
		col << upper_part, lower_part;
		jacobian.col(i + 1) = col;
	}

	return jacobian;
}

Eigen::MatrixXf Robot::getJacobian() const
{
	// jacobian for current joint angles
	return getJacobian(getJointAngles());
}

Eigen::VectorXf Robot::getJointSpeeds() const
{
	Eigen::VectorXf speeds(getNumJoints());

	for (int i = 0; i < links_.size(); i++) {
		speeds(i) = links_[i].getJointSpeed();
	}
	return speeds;
}

Eigen::VectorXf Robot::getJointAngles() const
{
	Eigen::VectorXf angles(getNumJoints());
	for (int i = 0; i < links_.size(); i++) {
		angles(i) = links_[i].getJointRotation();
	}
	return angles;
}

Eigen::VectorXf Robot::getTargetJointAngles() const
{
	Eigen::VectorXf angles(getNumJoints());
	for (int i = 0; i < links_.size(); i++) {
		angles(i) = links_[i].getJointTargetRotation();
	}
	return angles;
}
void Robot::setJointTargetAngles(Eigen::VectorXf angles) {
	assert(angles.rows() == getNumJoints());
	for (int i = 0; i < angles.rows(); i++) {
		links_[i].setJointTargetRotation(angles(i));
	}
}

void Robot::setJointTargetAngle(unsigned index, float angle)
{
	links_[index].setJointTargetRotation(angle);
}

void Robot::updateToWorldTransforms()
{
	Vec3f translation;
	Mat3f rotation;

	// TODO: this may not be necessary, if i make the joint update it's link matrix on its own every time an update cycle happens
	// first update the link matrix, since rotation may have changed
	for (int i = 0; i < links_.size(); i++) {
		links_[i].updateLinkMatrix(links_[i].getJointRotation());
	}

	// now chain the link transformations like so:
	// for link 0 the result should be W_T_0 (which is W_T_B * B_T_0)
	// for link 1: W_T_1
	// for link 2: W_T_2
	// etc...
	// index always tells which frame is the result of the transformation
	Mat4f current_transform = worldToBase_;
	for (int i = 0; i < links_.size(); i++) {
		current_transform *= links_[i].getLinkMatrix();
		links_[i].setToWorld(current_transform);
	}
}

std::vector<JointedLink> Robot::getLinks() const
{
	return links_;
}

void Robot::renderSkeleton() const
{
	for (const JointedLink& link : links_) {
		link.renderSkeleton();
	}
}

std::vector<Vertex> Robot::getMeshVertices() const
{
	std::vector<Vertex> allVerts;
	for (int i = 0; i < links_.size(); i++) {
		std::vector<Vertex> linkVertices = links_[i].getMeshVertices(i);
		allVerts.insert(allVerts.end(), linkVertices.begin(), linkVertices.end());
	}
	return allVerts;
}

std::vector<FW::Mat4f> Robot::getToWorldTransforms() const
{
	vector<Mat4f> transforms;
	for (int i = 0; i < links_.size(); i++) {
		transforms.push_back(links_[i].getToWorld());
	}
	return transforms;
}

void Robot::buildKinematicModel(const std::vector<DhParam>& dh_params)
{
	int link_index = 0;
	for (const DhParam& link_params : dh_params) {
		cout << "Parameters for link " << link_index << ":" << endl;
		cout << "	length: " << link_params.a << endl;
		cout << "	twist: " << link_params.alpha << endl;
		cout << "	offset: " << link_params.d << endl;
		cout << "	rotation: " << 0 << endl;

		JointedLink link(link_params, 0.0f);
		links_.push_back(link);
		link_index++;
	}
	cout << "Created " << links_.size() << " links" << endl;

	updateToWorldTransforms();

	cout << endl << "Manipulator Jacobian when joint angles are zero: " << endl;
	cout << getJacobian() << endl;
}
