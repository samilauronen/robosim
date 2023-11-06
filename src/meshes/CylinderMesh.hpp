#pragma once
#include <vector>

#include "Mesh.hpp"

class CylinderMesh : public Mesh {
public:
	CylinderMesh(float radius, float length, Eigen::Vector4f color, int subdivisions = 32);
private:
	std::vector<Vertex> createVertexRing(int numVerts, float z) const;
	std::vector<Vertex> createFaceTriangles(const std::vector<Vertex>& ringVertices, float z) const;

	float radius_;
	float length_;
	Eigen::Vector4f color_;
};