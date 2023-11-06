#include "JointedLinkMesh.hpp"
#include "CylinderMesh.hpp"
#include "BoxMesh.hpp"

using namespace Eigen;

const Vector4f JointedLinkMesh::JOINT_COLOR = Vector4f(0.05, 0.6, 0.4, 1.0);
const Vector4f JointedLinkMesh::LINK_COLOR = Vector4f(0.9, 0.5, 0, 1.0);


JointedLinkMesh::JointedLinkMesh(float z_len, float x_len, int link_number):
	Mesh()
{
	float init_link_thickness = 0.1f;
	float init_joint_radius = sqrt(2*(init_link_thickness / 2)* (init_link_thickness / 2)) + 0.001;
	float init_joint_length = 0.2f;
	float size_reduction_factor = 0.8;

	float reduction = pow(size_reduction_factor, link_number);
	float radius = init_joint_radius * reduction;
	float length = init_joint_length * reduction;
	float thickness = init_link_thickness * reduction;

	bool both_directions = z_len > 0.01 && x_len > 0.01;

	// positions where the different link ends are
	Vector3f x_link_end = Vector3f(x_len, 0, z_len);
	Vector3f z_link_end = Vector3f(0, 0, z_len);

	// if both links exist, make small adjustment to lengths so that they "interlock" nicely with eachother
	if (both_directions) {
		z_len += init_link_thickness * reduction / 2;
		x_len -= init_link_thickness * reduction / 2;
	}

	// create meshes
	// joint mesh is centered at origin and oriented along z
	// z_link starts from origin and ends at z_len along z
	BoxMesh z_link(thickness, thickness, z_len, LINK_COLOR);
	BoxMesh x_link(thickness, thickness, x_len, LINK_COLOR);
	CylinderMesh joint(radius, length, JOINT_COLOR);

	// move x_link to its correct position
	Vector3f i, j, k;
	Matrix3f link_orientation;
	k = (z_link_end - x_link_end).normalized();
	j = Vector3f(0, 1, 0);
	i = j.cross(k);
	link_orientation.col(0) = i;
	link_orientation.col(1) = j;
	link_orientation.col(2) = k;
	x_link.transform(Translation3f(x_link_end) * link_orientation);

	// compose all mesh vertices into one vector
	std::vector<Vertex> z_verts, x_verts, joint_verts;
	z_verts = z_link.getVertices();
	x_verts = x_link.getVertices();
	joint_verts = joint.getVertices();
	vertices_.insert(vertices_.end(), z_verts.begin(), z_verts.end());
	vertices_.insert(vertices_.end(), x_verts.begin(), x_verts.end());
	vertices_.insert(vertices_.end(), joint_verts.begin(), joint_verts.end());
}