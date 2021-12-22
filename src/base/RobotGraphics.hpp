#pragma once

#include "Robot.hpp"

namespace RobotGraphics {

const Vec3f JOINT_COLOR = Vec3f(0.05, 0.6, 0.4);
const Vec3f LINK_COLOR = Vec3f(0.9, 0.5, 0);

std::vector<Vertex> getMeshVertices(std::vector<Link> links);
void renderSkeleton(std::vector<Link> links);

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

} // namespace RobotGraphics
