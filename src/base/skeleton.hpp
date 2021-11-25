#pragma once

#include <base/Math.hpp>

#include <string>
#include <vector>
#include <map>

//static const unsigned WEIGHTS_PER_VERTEX = 8u;
//static const unsigned ANIM_JOINT_COUNT = 100u;
//struct AnimFrame
//{
//	FW::Vec3f position;
//	FW::Vec3f angles[ ANIM_JOINT_COUNT ];
//};

struct Joint
{
	// Joint rotation in Euler angles.
	// This changes when you animate the mesh.
	FW::Vec3f rotation;
	
	// Origin of current bone in parent's coordinate system.
	// This always stays fixed.
	FW::Vec3f position;

	// Current transform from joint space to parent joint's space.
	// (It is computed using the rotation and the position.)
	FW::Mat4f to_parent;

	// Current transform from joint space to world space.
	// (This is the matrix T_i in the lecture slides' notation.)
	FW::Mat4f to_world;

	// Transform from world space to joint space for the initial "bind" configuration.
	// (This is the matrix inv(B_i) in the lecture slides' notation)
	FW::Mat4f to_bind_joint;

	//
	std::string name;

	// Index of child joint (-1 if doesn't exist)
	int child;
	// Index of parent joint (-1 for root).
	int parent;
};

class Skeleton
{
public:
	int						getJointIndex(std::string name);
	std::string				getJointName(unsigned index) const;
	FW::Vec3f				getJointRotation(unsigned index) const;
	int						getJointParent(unsigned index) const;

	void					setJointRotation(unsigned index, FW::Vec3f euler_angles);
	void					incrJointRotation(unsigned index, FW::Vec3f euler_angles);

	void					addJoint();

	void					updateToWorldTransforms();
	float					normalizeScale();

	std::vector<FW::Mat4f>	getToWorldTransforms();
	size_t					getNumJoints() { return joints_.size(); }

private:
	void					updateToWorldTransforms(unsigned joint_index, const FW::Mat4f& parent_to_world);
	void					computeToBindTransforms();

	std::vector<Joint>		joints_;
	std::map<std::string, int> jointNameMap;
};
