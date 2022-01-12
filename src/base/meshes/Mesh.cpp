#include "Mesh.hpp"

Mesh::Mesh():
	to_world_(Eigen::Affine3f::Identity())
{
}

std::vector<Vertex> Mesh::getVertices() const {
	std::vector<Vertex> transformedVerts;
	for (const Vertex& v : vertices_) {
		Vertex newVert;
		newVert.position = to_world_ * v.position;
		newVert.normal = (to_world_.linear().inverse().transpose() * v.normal).normalized();
		newVert.color = v.color;
		transformedVerts.push_back(newVert);
	}
	return transformedVerts;
}

void Mesh::setToWorldTransform(Eigen::Affine3f trans) {
	to_world_ = trans;
}
void Mesh::transform(Eigen::Affine3f trans) {
	for (Vertex& v : vertices_) {
		v.position = trans * v.position;
		v.normal = (trans.linear().inverse().transpose() * v.normal).normalized();
	}
}
