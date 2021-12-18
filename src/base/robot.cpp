#include "Robot.hpp"
#include "Utility.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <assert.h>

using namespace std;

// TODO:
/*
* Don't create meshes again for every frame, that just sucks! Make RobotGraphics more oop style
* Convert all maths to using eigen
* Use GLFW or some other window manager
* Use ImGui for UI
* Make your own camera class, or use an existing one (from GLFW?)
* Make prismatic joints possible?
* Inverse kinematics, given a point (first programmatically, later by mouse click in the world)
* Maybe have a toggle for drawing a wireframe-ball around the robot, visualizing it's range
* inverse jacobians!, for trajectories!
* Dynamics???
* ImGui for more awesome ui?
*/

Robot::Robot(std::string dh_param_filename, FW::Vec3f location)
	: selected_joint_(0)
{
	// base to 0 frame
	Mat3f rotation = Mat3f::rotation(Vec3f(1, 0, 0), -FW_PI / 2);
	baseToZero_ = combineToMat4f(rotation, Vec3f(0, 0, 0));

	// set the transformation from world to base, no rotation here, just set the location
	worldToBase_ = combineToMat4f(Mat3f::rotation(Vec3f(0, 0, 0), 0), location);

	// create kinematic model and links of the robot
	vector<DhParam> params = loadDhParams(dh_param_filename);
	buildKinematicModel(params);
}

void Robot::update(float dt_millis)
{
	// in rad/s
	const float JOINT_SPEED = FW_PI / 3;

	for (int i = 1; i < links_.size(); i++) {
		float target = getTargetJointAngle(i);
		float current = getJointAngle(i);

		float diff = current - target;

		if (FW::abs(diff) > 0.01) {
			links_[i].is_moving = true;
			if (current < target) {
				links_[i].joint_speed = JOINT_SPEED;
				links_[i].rotation += links_[i].joint_speed * (dt_millis / 1000);
				
			}
			else {
				links_[i].joint_speed = -JOINT_SPEED;
				links_[i].rotation += links_[i].joint_speed * (dt_millis / 1000);
			}
		}
		else {
			links_[i].is_moving = false;
			links_[i].joint_speed = 0;
		}
	}
	updateToWorldTransforms();
}

FW::Vec3f Robot::getTcpPosition() const
{
	return links_[links_.size() - 1].to_world * Vec3f(0, 0, 0);
}

Eigen::VectorXf Robot::getTcpSpeed() const
{
	Eigen::MatrixXf jacobian = getJacobian();
	Eigen::VectorXf joint_speeds = getJointSpeeds();
	assert(jacobian.cols() == joint_speeds.rows());  // required for multiplication

	return jacobian * joint_speeds;
}

std::vector<float> Robot::inverseKinematics(FW::Vec3f tcp_pos) const
{
	return std::vector<float>();
}

const Eigen::MatrixXf Robot::getJacobian() const
{
	int n_rows = 6;					// jacobian will always have 6 rows, since 3D space has 6 degrees of freedom
	int n_cols = getNumJoints();	// number of columns is the number of joint angles that can be controlled

	Eigen::MatrixXf jacobian(n_rows, n_cols);

	// returns the transform 0_T_i as Eigen::Matrix4f
	auto get_zero_to_i = [&](int i) {
		Mat4f fwres;
		for (int j = 0; j <= i; j++) {
			fwres *= links_[j].link_matrix;
		}

		Eigen::Matrix4f res;
		Vec4f row0 = fwres.getRow(0);
		Vec4f row1 = fwres.getRow(1);
		Vec4f row2 = fwres.getRow(2);
		Vec4f row3 = fwres.getRow(3);

		res <<  row0.x, row0.y, row0.z, row0.w,
				row1.x, row1.y, row1.z, row1.w,
				row2.x, row2.y, row2.z, row2.w,
				row3.x, row3.y, row3.z, row3.w;
		return res;
	};

	// https://automaticaddison.com/the-ultimate-guide-to-jacobian-matrices-for-robotics/
	Eigen::Matrix4f zero_to_n = get_zero_to_i(links_.size() - 1);
	Eigen::Vector3f transl_0_to_n = zero_to_n.col(3).head<3>();

	// transformations from frame zero to frame i
	Eigen::Matrix4f tf_0_to_i;
	Eigen::Matrix3f rot_0_to_i;
	Eigen::Vector3f transl_0_to_i;

	Eigen::Vector3f z_vec = Eigen::Vector3f(0, 0, 1);

	// fill the matrix column-by-column
	for (int i = 0; i < getNumJoints(); i++) {
		tf_0_to_i = get_zero_to_i(i);
		rot_0_to_i = tf_0_to_i.block<3, 3>(0, 0);
		transl_0_to_i = tf_0_to_i.block<3, 1>(0, 3);

		Eigen::Vector3f upper_part = (rot_0_to_i * z_vec).cross(transl_0_to_n - transl_0_to_i);
		Eigen::Vector3f lower_part = rot_0_to_i * z_vec;

		Eigen::Matrix<float, 6, 1> col;
		col << upper_part, lower_part;
		jacobian.col(i) = col;
	}

	return jacobian;
}

Eigen::VectorXf Robot::getJointSpeeds() const
{
	Eigen::VectorXf speeds(getNumJoints());

	for (int i = 1; i < links_.size(); i++) {
		speeds(i - 1) = links_[i].joint_speed;
	}

	return speeds;
}

Eigen::VectorXf Robot::getJointAngles() const
{
	Eigen::VectorXf angles(getNumJoints());
	for (int i = 1; i < links_.size(); i++) {
		angles(i) = links_[i].rotation;
	}
	return angles;
}

Eigen::VectorXf Robot::getTargetJointAngles() const
{
	Eigen::VectorXf angles(links_.size());
	for (int i = 0; i < links_.size(); i++) {
		angles(i) = links_[i].target_rotation;
	}
	return angles;
}

void Robot::setJointTargetAngle(unsigned index, float angle)
{
	links_[index].target_rotation = angle;
}

void Robot::incrJointAngle(unsigned index, float angle)
{
	links_[index].rotation += angle;
	updateToWorldTransforms();
}

void Robot::setJointAngle(unsigned index, float angle)
{
	links_[index].rotation = angle;
	updateToWorldTransforms();
}

void Robot::updateToWorldTransforms()
{
	Vec3f translation;
	Mat3f rotation;

	// first update the link matrix, since rotation may have changed
	for (int i = 0; i < links_.size(); i++) {
		Link& j = links_[i];

		// https://www.youtube.com/watch?v=nuB_7BkYNMk
		translation = Vec3f(0, 0, j.p.d);						// first move up so that we are at the level of the common normal
		rotation = Mat3f::rotation(Vec3f(0, 0, 1), j.rotation);	// then rotate around z to align x with next x

		Mat4f z_screw = combineToMat4f(rotation, translation);  // translation and rotation with respect to z

		// after applying the z-screw, we are at the correct level and our x-axis points in the correct direction
		// now move towards our new x by link length
		translation = Vec3f(j.p.a, 0, 0);
		rotation = Mat3f::rotation(Vec3f(1, 0, 0), j.p.alpha);	// apply link twist (rotate around x-axis)

		Mat4f x_screw = combineToMat4f(rotation, translation);	// translation and rotation with respect to x

		Mat4f link_matrix = z_screw * x_screw;					// resulting link matrix

		j.z_screw = z_screw;
		j.x_screw = x_screw;
		j.link_matrix = link_matrix;
	}

	// now chain the link transformations like so:
	// for link 0 the result should be W_T_0 (which is W_T_B * B_T_0)
	// for link 1: W_T_1
	// for link 2: W_T_2
	// etc...
	// index always tells which frame is the result of the transformation
	Mat4f current_transform = worldToBase_ * baseToZero_;
	for (int i = 0; i < links_.size(); i++) {
		current_transform *= links_[i].link_matrix;
		links_[i].to_world = current_transform;
	}
}

const std::vector<Link>& Robot::getLinks() const
{
	return links_;
}

std::vector<FW::Mat4f> Robot::getToWorldTransforms() const
{
	vector<Mat4f> transforms;
	for (int i = 0; i < links_.size(); i++) {
		transforms.push_back(links_[i].to_world);
	}
	return transforms;
}

std::vector<DhParam> Robot::loadDhParams(const std::string filename)
{
	ifstream file_input(filename, ios::in);
	string line;

	std::vector<DhParam> params;

	while (getline(file_input, line)) {
		if (line[0] == '#') continue;

		stringstream ss(line);
		DhParam p;
		string sink;
		string alpha_str;

		ss >> p.sigma;

		if (p.sigma == "R") {
			ss >> p.a >> p.d >> alpha_str >> sink;
			float result;

			float operand1, operand2;
			char operation;
			size_t operator_loc;

			auto set_operands = [&](const char operation) {
				if (operator_loc != string::npos) {
					string before = alpha_str.substr(0, operator_loc);
					string after = alpha_str.substr(operator_loc + 1, alpha_str.length() - operator_loc);

					if (before.find("pi") != string::npos) {
						operand1 = FW_PI;
						operand2 = std::stof(after);
					}
					else if (after.find("pi") != string::npos) {
						operand1 = std::stof(before);
						operand2 = FW_PI;
					}
					else {
						cout << "wtf!!! " << endl;
					}
				}
			};
			if ((operator_loc = alpha_str.find('*')) != string::npos) {
				set_operands('*');
				result = operand1 * operand2;
			}
			else if ((operator_loc = alpha_str.find('/')) != string::npos) {
				set_operands('/');
				result = operand1 / operand2;
			}
			else {
				result = 0.0f;
			}

			p.alpha = result;
			p.theta = -1;
		} else if (p.sigma == "P") {
			ss >> p.a >> sink >> p.alpha >> p.theta;
			p.d = -1;
		}
		else {
			cout << "Invalid joint type: " << p.sigma << endl;
			continue;
		}
		
		params.push_back(p);
	}
	return params;
}

void Robot::buildKinematicModel(const std::vector<DhParam>& params)
{
	Link link_zero;
	link_zero.p = DhParam{};
	link_zero.rotation = 0;
	link_zero.target_rotation = 0;
	link_zero.joint_speed = 0;
	link_zero.is_locked = true;	// should not be rotated
	links_.push_back(link_zero);

	int link_index = 1;
	for (const DhParam& p : params) {

		cout << "Parameters for link " << link_index << ":" << endl;
		cout << "	length: " << p.a << endl;
		cout << "	twist: " << p.alpha << endl;
		cout << "	offset: " << p.d << endl;
		cout << "	rotation: " << 0 << endl;

		Link l;
		l.p = p;
		l.rotation = 0;
		l.target_rotation = 0;
		l.joint_speed = 0;
		links_.push_back(l);
		link_index++;
	}
	cout << "Created " << links_.size() << " links" << endl;

	updateToWorldTransforms();

	cout << endl << "Manipulator Jacobian when joint angles are zero: " << endl;
	cout << getJacobian() << endl;
}
