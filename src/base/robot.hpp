#pragma once
#include <base/Math.hpp>
#include <vector>
#include <string>
#include <iostream>

using namespace FW;



struct Vertex
{
	Vec3f position;
	Vec3f normal;
	Vec3f color;
};

struct DhParam {
	// d = offset
	// a = link length
	// alpha = twist
	// theta = joint angle
	float d, a, alpha, theta;

	// type of joint: 'R' = revolute, 'P' = prismatic
	std::string sigma;
};

struct Joint {
	// Joint rotation in Euler angles.
	float rotation;

	//params
	DhParam p;

	// Joint position in parent coordinates.
	FW::Vec3f position;

	// Current transform from joint space to parent joint's space.
	// FW::Mat4f to_parent;

	// Current transform from joint space to world space.
	FW::Mat4f to_world;

	// testing
	FW::Mat4f to_parent;
};

class Robot {
	 
public:
	Robot(std::string dh_param_filename, Vec3f location);

	// setting TCP transformation
	const Mat4f& getTcpTransform() const { return tool; };
	void setTcpTransform(const Mat4f& newTransform) { tool = newTransform; };

	// get or set base location and orientation in the world
	const Mat4f& getBaseToWorld() const { return baseToWorld_; };
	void setBaseToWorld(const Mat4f& newBTW) { baseToWorld_ = newBTW; };

	// for rendering
	void renderSkeleton();
	std::vector<Vertex> getMeshVertices();

	// skeleton-ish functionality
	FW::Vec3f				getJointRotation(unsigned index) const { return joints_[index].rotation; };
	void					setJointRotation(unsigned index, float angle) { joints_[index].rotation = angle; };
	void					incrJointRotation(unsigned index, float angle) { joints_[index].rotation += angle; };

	FW::Vec3f				getJointRotation() const { return joints_[selected_joint_].rotation; };
	void					setJointRotation(float angle) { joints_[selected_joint_].rotation = angle; };
	void					incrJointRotation(float angle) { joints_[selected_joint_].rotation += angle; };

	void					setSelectedJoint(unsigned index) { selected_joint_ = index; };
	unsigned				getSelectedJoint() { return selected_joint_; };
	
	void					updateToWorldTransforms();
	std::vector<FW::Mat4f>	getToWorldTransforms();

	size_t					getNumJoints() { return joints_.size(); };

private:
	Mat4f tool;
	Mat4f baseToWorld_;
	Mat4f baseToZero_;

	void loadDhParams();
	void buildModel();

	std::vector<Joint> joints_;
	std::vector<DhParam> params_;
	std::string filename_;

	// for drawing and control
	unsigned selected_joint_;
};

// box-shaped mesh representing a link between joints
class LinkMesh {
public:
	LinkMesh(float thickness, float length) { thickness_ = thickness; length_ = length; };

	std::vector<Vertex> getVertices();
private:
	float thickness_;
	float length_;
};


// cylinder mesh representing a rotational joint
class JointMesh {
public:
	JointMesh(float radius, float depth) { radius_ = radius; depth_ = depth; };

	std::vector<Vertex> getVertices();
private:
	std::pair<std::vector<Vertex>, std::vector<Vertex>> makeFace(int numRingVerts, float z);

	float radius_;
	float depth_;
};