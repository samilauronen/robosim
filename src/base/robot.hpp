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

	// when true, the rotation cannot be changed
	bool is_locked;

	// dh parameters of this link
	DhParam p;

	// calculated from the DH parameters of this link
	// relates this link's frame to the frame of the previous link
	FW::Mat4f link_matrix;

	// transforms the coordinates of this link frame to world frame
	// created by multiplying together all link matrices of the previous links and the current link
	FW::Mat4f to_world;
};

class Robot {
	 
public:
	Robot(std::string dh_param_filename, Vec3f location);

	// get or set base location and orientation in the world
	const Mat4f& getWorldToBase() const { return worldToBase_; };
	void setWorldToBase(const Mat4f& newWTB) { worldToBase_ = newWTB; };

	// for rendering
	void renderSkeleton();
	std::vector<Vertex> getMeshVertices();

	// skeleton-ish functionality
	FW::Vec3f				getJointRotation(unsigned index) const { return links_[index].rotation; };
	void					setJointRotation(unsigned index, float angle) { links_[index].rotation = angle; };
	void					incrJointRotation(unsigned index, float angle) { links_[index].rotation += angle; };

	FW::Vec3f				getJointRotation() const { return links_[selected_joint_].rotation; };
	void					setJointRotation(float angle) { links_[selected_joint_].rotation = angle; };
	void					incrJointRotation(float angle) { links_[selected_joint_].rotation += angle; };

	void					setSelectedJoint(unsigned index) { selected_joint_ = index; };
	unsigned				getSelectedJoint() { return selected_joint_; };
	
	void							updateToWorldTransforms();
	std::vector<FW::Mat4f>			getToWorldTransforms();
	const std::vector<Link>&		getLinks();

	size_t					getNumJoints() { return links_.size() - 1;  /* zeroth link is not counted */ };


private:
	Mat4f tool;
	Mat4f worldToBase_;
	Mat4f baseToZero_;

	void loadDhParams();
	void buildModel();

	std::vector<Link> links_;
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