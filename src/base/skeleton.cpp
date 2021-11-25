#include "skeleton.hpp"
#include "utility.hpp"

#include <cassert>
#include <fstream>
#include <stack>
#include <sstream>

using namespace std;
using namespace FW;


void Skeleton::setJointRotation(unsigned index, Vec3f euler_angles) {
	Joint& joint = joints_[index];

	// For convenient reading, we store the rotation angles in the joint
	// as-is, in addition to setting the to_parent matrix to match the rotation.
	joint.rotation = euler_angles;

	joint.to_parent = Mat4f();
	joint.to_parent.setCol(3, Vec4f(joint.position, 1.0f));

	// YOUR CODE HERE (R2)
	// Modify the "to_parent" matrix of the joint to match
	// the given rotation Euler angles. Thanks to the line above,
	// "to_parent" already contains the correct transformation
	// component in the last column. Compute the rotation that
	// corresponds to the current Euler angles and replace the
	// upper 3x3 block of "to_parent" with the result.
	// Hints: You can use Mat3f::rotation() three times in a row,
	// once for each main axis, and multiply the results.

	// rotation order in this case: Z * Y * X
	Mat3f rotation =
		Mat3f::rotation(Vec3f(0, 0, 1), euler_angles.z) *
		Mat3f::rotation(Vec3f(0, 1, 0), euler_angles.y) *
		Mat3f::rotation(Vec3f(1, 0, 0), euler_angles.x);

	joint.to_parent.setCol(0, Vec4f(rotation.getCol(0), 0));
	joint.to_parent.setCol(1, Vec4f(rotation.getCol(1), 0));
	joint.to_parent.setCol(2, Vec4f(rotation.getCol(2), 0));

}

void Skeleton::incrJointRotation(unsigned index, Vec3f euler_angles) {
	setJointRotation(index, getJointRotation(index) + euler_angles);
}

void Skeleton::addJoint()
{
	Joint j;
	j.rotation = Vec3f(0, 0, 0);
	j.position = Vec3f(0, 0, 0);
	j.parent = -1;
	j.child = -1;
	j.name = "new joint";

	joints_.push_back(j);
}

void Skeleton::updateToWorldTransforms() {
	// Here we just initiate the hierarchical transformation from the root node (at index 0)
	// and an identity transformation, precisely as in the lecture slides on hierarchical modeling.
	updateToWorldTransforms(0, Mat4f());

}

void Skeleton::updateToWorldTransforms(unsigned joint_index, const Mat4f& parent_to_world) {
	// YOUR CODE HERE (R1)
	// Update transforms for joint at joint_index and its children.
	
	// visit
	// current joint to world = parent joint to world trasform *  current joint to parent transform
	Mat4f curr_to_world;
	int child_index = -1;
	if (joints_.size() != 0) {
		curr_to_world = parent_to_world * joints_[joint_index].to_parent;
		joints_[joint_index].to_world = curr_to_world;
		child_index = joints_[joint_index].child;
	}
	
	// continue to child if it exists
	if (child_index != -1) {
		updateToWorldTransforms(child_index, curr_to_world);
	}
}

void Skeleton::computeToBindTransforms() {
	updateToWorldTransforms();
	// YOUR CODE HERE (R4)
	// Given the current to_world transforms for each bone,
	// compute the inverse bind pose transformations (as per the lecture slides),
	// and store the results in the member to_bind_joint of each joint.
	for (Joint& j : joints_) {
		j.to_bind_joint = j.to_world.inverted();
	}
}

vector<Mat4f> Skeleton::getToWorldTransforms() {
	updateToWorldTransforms();
	vector<Mat4f> transforms;
	for (const auto& j : joints_)
		transforms.push_back(j.to_world);
	return transforms;
}

// Make sure the skeleton is of sensible size.
// Here we just search for the largest extent along any axis and divide by twice
// that, effectively causing the model to fit in a [-0.5, 0.5]^3 cube.
float Skeleton::normalizeScale()
{
	float scale = 0;

	for (auto& j : joints_)
		scale = FW::max(scale, abs(j.position).max());
	scale *= 2;
	for (auto& j : joints_)
		j.position /= scale;

	return scale;
}

string Skeleton::getJointName(unsigned index) const {
	return joints_[index].name;
}

Vec3f Skeleton::getJointRotation(unsigned index) const {
	return joints_[index].rotation;
}

int Skeleton::getJointParent(unsigned index) const {
	return joints_[index].parent;
}

int Skeleton::getJointIndex(string name) {
	return jointNameMap[name];
}
