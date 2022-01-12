#pragma once
#include "Mesh.hpp"

// graphical representation for JointedLink
class JointedLinkMesh : public Mesh {
public:
	JointedLinkMesh(float z_len, float x_len, int link_number);

	static const FW::Vec3f JOINT_COLOR;
	static const FW::Vec3f LINK_COLOR;
};