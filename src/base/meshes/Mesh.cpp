#include "Mesh.hpp"

std::vector<Vertex> Mesh::getVertices() const {
	std::vector<Vertex> transformedVerts;
	for (const Vertex& v : vertices_) {
		Vertex newVert;
		newVert.position = to_world_ * v.position;
		newVert.normal = to_world_.getXYZ().inverted().transposed() * v.normal;
		newVert.color = v.color;
		transformedVerts.push_back(newVert);
	}
	return transformedVerts;
}

void Mesh::setToWorldTransform(FW::Mat4f trans) {
	to_world_ = trans;
}
void Mesh::transform(FW::Mat4f trans) {
	for (Vertex& v : vertices_) {
		v.position = trans * v.position;
		v.normal = trans.getXYZ().inverted().transposed() * v.normal;
	}
}
