#pragma once

#include <vector>

#include "Mesh.hpp"

class SphereMesh : public Mesh {
public:
	SphereMesh(float radius, Eigen::Vector3f color, int angle_subdivisions = 20);
private:
	std::vector<Vertex> createTrianglesToVertex(const std::vector<Vertex>& ringVertices, const Vertex center) const;
	std::vector<Vertex> createTrianglesBetweenRings(const std::vector<Vertex>& ring1_vertices, const std::vector<Vertex>& ring2_vertices) const;

	float radius_;
	Eigen::Vector3f color_;
};