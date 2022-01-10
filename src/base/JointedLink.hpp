#pragma once

#include <vector>

#include "base/Math.hpp"
#include "DhParam.hpp"

// TODO: move this to some file like "GraphicsPrimitives" or something
// or actually, maybe not neede at all once we have gluCylinders and somesuch in use
struct Vertex
{
	FW::Vec3f position;
	FW::Vec3f normal;
	FW::Vec3f color;
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

// describes a combination of a joint and a link
class JointedLink {
public:
	JointedLink(DhParam params, float joint_rotation);

	// used to evaluate the link matrix for any joint angle state
	// does not modify current state of the struct
	// used by iterative inverse kinematics solvers
	FW::Mat4f evalLinkMatrix(float rotationAngle) const;

	// evalueates link matrix at rotation angle and sets current link matrix to be that
	void updateLinkMatrix(float rotationAngle);

	// setters
	void setToWorld(FW::Mat4f to_world) { to_world_ = to_world; };
	void setJointRotation(float rotation_angle) { rotation_ = rotation_angle; };
	void setJointSpeed(float new_speed) { joint_speed_ = new_speed; };
	void setJointTargetRotation(float target_angle) { target_rotation_ = target_angle; };

	// matching getters
	FW::Mat4f getToWorld() const { return to_world_; };
	float getJointRotation() const { return rotation_; };
	float getJointSpeed() const { return joint_speed_; };
	float getJointTargetRotation() const { return target_rotation_; };

	// no setter
	DhParam getDhParams() const { return params_; };
	FW::Mat4f getLinkMatrix() const { return link_matrix_; };

	// graphics:
	static const FW::Vec3f JOINT_COLOR;
	static const FW::Vec3f LINK_COLOR;

	// returns the vertices of the mesh that represents the combination of a joint and a link
	std::vector<Vertex> getMeshVertices(int i) const;
	void renderSkeleton() const;
private:
	DhParam params_;		// DH parameters of this link

	float joint_speed_;     // current speed of the joint in rad/s
	float rotation_;		// rotation angle of the joint moving this link
	float target_rotation_;	// target rotation for the joint, current rotation will move towards this when update() is called

	// combination of applying z_screw and  then x_screw
	// relates this link's frame to the frame of the previous link
	FW::Mat4f link_matrix_;

	// transforms the coordinates of this link frame to world frame
	// created by multiplying together all link matrices of the previous links and the current link
	FW::Mat4f to_world_;
};