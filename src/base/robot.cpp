#include "Robot.hpp"
#include "Utility.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

using namespace std;

// TODO:
/*
* Don't create meshes again for every frame, that just sucks! Make RobotGraphics more oop style
* Make it possible to write "pi" divided or multiplied by somthing in the dh params (implement parsing of that)
* Make prismatic joints possible?
* Inverse kinematics, given a point (first programmatically, later by mouse click in the world)
* Maybe have a toggle for drawing a wireframe-ball around the robot, visualizing it's range
* Jacobians, especially inverse, for trajectories!
* Dynamics???
* ImGui for more awesome ui?
*/

Robot::Robot(std::string dh_param_filename, FW::Vec3f location)
	: selected_joint_(0),
	filename_(dh_param_filename)
{
	// base to 0 frame
	Mat3f rotation = Mat3f::rotation(Vec3f(1, 0, 0), -FW_PI / 2);
	baseToZero_ = combineToMat4f(rotation, Vec3f(0, 0, 0));

	loadDhParams();
	buildModel();

	// set the transformation from world to base, no rotation here, just set the location
	worldToBase_ = combineToMat4f(Mat3f::rotation(Vec3f(0,0,0),0), location);

}

FW::Vec3f Robot::getTcpPosition()
{
	updateToWorldTransforms();
	return links_[links_.size() - 1].to_world * Vec3f(0, 0, 0);
}

std::vector<float> Robot::inverseKinematics(FW::Vec3f tcp_pos)
{
	return std::vector<float>();
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
	Mat4f previous_transform;
	Mat4f current_transform;
	previous_transform = worldToBase_ * baseToZero_;
	for (int i = 0; i < links_.size(); i++) {
		current_transform = previous_transform * links_[i].link_matrix;
		links_[i].to_world = current_transform;
		previous_transform = current_transform;
	}
}

const std::vector<Link>& Robot::getLinks()
{
	updateToWorldTransforms();
	return links_;
}

std::vector<FW::Mat4f> Robot::getToWorldTransforms()
{
	updateToWorldTransforms();
	vector<Mat4f> transforms;
	for (int i = 0; i < links_.size(); i++) {
		transforms.push_back(links_[i].to_world);
	}
	return transforms;
}

void Robot::loadDhParams()
{
	ifstream file_input(filename_, ios::in);
	string line;

	while (getline(file_input, line)) {
		if (line[0] == '#') continue;

		stringstream ss(line);
		DhParam p;
		string sink;

		ss >> p.sigma;

		if (p.sigma == "R") {
			ss >> p.a >> p.d >> p.alpha >> sink;
			p.theta = -1;
		} else if (p.sigma == "P") {
			ss >> p.a >> sink >> p.alpha >> p.theta;
			p.d = -1;
		}
		else {
			cerr << "Invalid joint type: " << p.sigma << endl;
			continue;
		}
		
		params_.push_back(p);
	}
}

void Robot::buildModel()
{
	Link link_zero;
	link_zero.p = DhParam{};
	link_zero.rotation = 0;
	link_zero.is_locked = true;	// should not be rotated
	links_.push_back(link_zero);

	int link_index = 1;
	for (DhParam p : params_) {

		cerr << "Parameters for link " << link_index << ":" << endl;
		cerr << "	length: " << p.a << endl;
		cerr << "	twist: " << p.alpha << endl;
		cerr << "	offset: " << p.d << endl;
		cerr << "	rotation: " << 0 << endl;

		Link l;
		l.p = p;
		l.rotation = 0;
		links_.push_back(l);
		link_index++;
	}
	cerr << "Created " << links_.size() << " links";
}
