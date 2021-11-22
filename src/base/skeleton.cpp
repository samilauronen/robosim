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

void Skeleton::updateToWorldTransforms() {
	// Here we just initiate the hierarchical transformation from the root node (at index 0)
	// and an identity transformation, precisely as in the lecture slides on hierarchical modeling.
	if (!animationMode)
		updateToWorldTransforms(0, Mat4f());
	else
		// If we're running an animation, we need to set all of the joint rotations of the current frame
		// and translate the root so it matches the animation.
		setAnimationState();
}

void Skeleton::updateToWorldTransforms(unsigned joint_index, const Mat4f& parent_to_world) {
	// YOUR CODE HERE (R1)
	// Update transforms for joint at joint_index and its children.
	
	// visit
	// current joint to world = parent joint to world trasform *  current joint to parent transform
	Mat4f curr_to_world = parent_to_world * joints_[joint_index].to_parent;
	joints_[joint_index].to_world = curr_to_world;

	// recurse to children, left to right
	for (int child_index : joints_[joint_index].children) {
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

vector<Mat4f> Skeleton::getSSDTransforms() {
	updateToWorldTransforms();
	// YOUR CODE HERE (R4)
	// Compute the relative transformations between the bind pose and current pose,
	// store the results in the vector "transforms". These are the transformations
	// passed into the actual skinning code. (In the lecture slides' terms,
	// these are the T_i * inv(B_i) matrices.)

	vector<Mat4f> transforms(joints_.size());

	for (size_t i = 0; i < joints_.size(); i++) {
		Joint j = joints_[i];
		transforms[i] = j.to_world * j.to_bind_joint;
	}
	
	return transforms;
}

float Skeleton::loadBVH(string skeleton_file) {
	ifstream in(skeleton_file);

	std::vector<Vec3i> axisPermutation;

	string s;
	string line;
	while (getline(in, line))
	{
		stringstream stream(line);
		stream >> s;
		
		if (s == "ROOT")
		{
			string jointName;
			stream >> jointName;
			loadJoint(in, -1, jointName, axisPermutation);
		}
		else if (s == "MOTION")
		{
			loadAnim(in, axisPermutation);
		}
	}

	float scale = normalizeScale();

	// initially set to_parent matrices to identity
	for (auto j = 0u; j < joints_.size(); ++j)
		setJointRotation(j, Vec3f(0, 0, 0));

	// this needs to be done while skeleton is still in bind pose
	computeToBindTransforms();

	return scale;
}

void Skeleton::loadJoint(ifstream& in, int parent, string name, std::vector<Vec3i>& axisPermutation)
{
	Joint j;
	j.name = name;
	j.parent = parent;

	bool pushed = false;

	int curIdx = -1;
	string s;
	string line;
	while (getline(in, line))
	{
		stringstream stream(line);
		stream >> s;
		if (s == "JOINT")
		{
			string jointName;
			stream >> jointName;
			loadJoint(in, curIdx, jointName, axisPermutation);
		}
		else if (s == "End")
		{
			while (getline(in, line))
			{
				// Read End block so it doesn't get interpreted as a closing bracket or offset keyword
				stringstream end(line);
				end >> s;
				if (s == "}")
					break;
			}
		}
		else if (s == "}")
		{
			return;
		}
		else if (s == "CHANNELS")
		{
			int channelCount;
			stream >> channelCount;
			if (channelCount == 6)
				stream >> s >> s >> s;

			Vec3i permutation;
			for (int i = 0; i < 3; ++i)
			{
				stream >> s;
				if (s == "Xrotation")
					permutation[0] = i;
				else if (s == "Yrotation")
					permutation[1] = i;
				else if (s == "Zrotation")
					permutation[2] = i;
			}
			axisPermutation.push_back(permutation);
		}
		else if (s == "OFFSET")
		{
			Vec3f pos;
			stream >> pos.x >> pos.y >> pos.z;
			j.position = pos;

			if (!pushed)
			{
				pushed = true;
				joints_.push_back(j);
				curIdx = int(joints_.size() - 1);

				if (parent != -1)
					joints_[parent].children.push_back(curIdx);
			}
			jointNameMap[name] = curIdx;
		}
	}
}

void Skeleton::loadAnim(ifstream& in, std::vector<Vec3i>& axisPermutation)
{
	string word;
	in >> word;
	int frames;
	in >> frames;

	animationData.resize(frames);

	int frameNum = 0;
	string line;

	// Discard unused lines
	getline(in, line);
	getline(in, line);

	// Load animation angle and position data for each frame
	while (getline(in, line))
	{
		float* frameData = (float*)(animationData.data() + frameNum);
		stringstream stream(line);
		int i = 0;
		while(stream.good())
			stream >> frameData[i++];
		frameNum++;
	}

	// Permute angle axes
	Vec3f posAccum;
	for (auto& f : animationData)
	{
		posAccum += f.position;
		for (int i = 0; i < ANIM_JOINT_COUNT; ++i)
		{
			Vec3f angles = f.angles[i];
			f.angles[i] = Vec3f(angles[axisPermutation[i].x], angles[axisPermutation[i].y], angles[axisPermutation[i].z]);
		}
	}

	// Offset position so that average stays at origin
	for (auto& f : animationData)
		f.position -= posAccum / float(animationData.size());
}

void Skeleton::load(string skeleton_file) {	
	ifstream in(skeleton_file);
	Joint joint;
	Vec3f pos;
	int parent;
	string name;
	unsigned current_joint = 0;
	while (in.good()) {
		in >> pos.x >> pos.y >> pos.z >> parent >> name;
		joint.name = name;
		joint.parent = parent;
		// set position of joint in parent's space
		joint.position = pos;
		if (in.good()) {
			joints_.push_back(joint);
			jointNameMap[name] = int(joints_.size() - 1);
			if (current_joint == 0) {
				assert(parent == -1 && "first node read should always be the root node");
			} else {
				assert(parent != -1 && "there should not be more than one root node");
				joints_[parent].children.push_back(current_joint);
			}
			++current_joint;
		}
	}

	// initially set to_parent matrices to identity
	for (auto j = 0u; j < joints_.size(); ++j)
		setJointRotation(j, Vec3f(0, 0, 0));

	// this needs to be done while skeleton is still in bind pose
	computeToBindTransforms();
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
	for (auto& f : animationData)
		f.position /= scale;

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

void Skeleton::setAnimationFrame(int AnimationFrame)
{
	animationFrame = AnimationFrame;
	animationMode = true;
}

void Skeleton::setAnimationState()
{
	// No actual animation exists.
	if (!animationData.size()) {
		animationMode = false;
		updateToWorldTransforms(0, Mat4f());
		return;
	}

	// Get the current position in the animation..
	int frame = animationFrame % animationData.size();
	auto& frameData = animationData[frame];
	// .. and set all joint rotations accordingly.
	for (int j = 0; j < ANIM_JOINT_COUNT; ++j)
		setJointRotation(j, frameData.angles[j] * FW_PI / 180.0f);

	// Also translate the root to the position given in the animation description.
	updateToWorldTransforms(0, Mat4f::translate(frameData.position));
}
