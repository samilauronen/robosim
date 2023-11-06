#define _USE_MATH_DEFINES
#include <math.h>

#include "CylinderMesh.hpp"

using namespace Eigen;

CylinderMesh::CylinderMesh(float radius, float length, Vector4f color, int subdivisions):
	Mesh(),
	color_(color),
	radius_(radius),
	length_(length)
{
	vertices_.clear();

	int num_ring_verts = subdivisions;

	// first face to z = -length/2
	std::vector<Vertex> ring1 = createVertexRing(num_ring_verts, -length_ / 2);
	std::vector<Vertex> face1 = createFaceTriangles(ring1, -length_ / 2);
	vertices_.insert(vertices_.end(), face1.begin(), face1.end());

	// second face to z = length/2
	std::vector<Vertex> ring2 = createVertexRing(num_ring_verts, length_ / 2);
	std::vector<Vertex> face2 = createFaceTriangles(ring2, length_ / 2);
	vertices_.insert(vertices_.end(), face2.begin(), face2.end());

	// now make the lateral surface
	for (int i = 0; i < num_ring_verts; i++) {
		// make two triangles in the reactangle formed by two pairs of vertices on the opposite faces
		Vertex v1, v2, v3, v4;

		v1 = ring1[i];
		v2 = ring1[(i + 1) % num_ring_verts];

		v3 = ring2[i];
		v4 = ring2[(i + 1) % num_ring_verts];

		vertices_.insert(vertices_.end(), { v1,v2,v4 ,v1,v3,v4 });
	}
}

std::vector<Vertex> CylinderMesh::createVertexRing(int num_vertices, float z) const {
	// generate ring of vertices around center point
	// x = r cos t, y= r sin t
	std::vector<Vertex> ring_vertices;
	for (float t = 0; t < 2 * M_PI; t += 2 * M_PI / num_vertices) {
		Vertex v;
		v.position = Vector3f(radius_ * cos(t), radius_ * sin(t), z);
		v.normal = v.position.normalized();
		v.color = color_;
		ring_vertices.push_back(v);
	}
	return ring_vertices;
}

std::vector<Vertex> CylinderMesh::createFaceTriangles(const std::vector<Vertex>& ring_vertices, float z) const {
	std::vector<Vertex> result;

	// middle vertex
	Vertex v0;
	v0.position = Vector3f(0, 0, z);
	v0.normal = v0.position.normalized();
	v0.color = color_;

	// for each pair of ring vertices, create triangle from it to the center vertex
	for (int i = 0; i < ring_vertices.size(); i += 1) {
		Vertex v1 = ring_vertices[i];
		Vertex v2 = ring_vertices[(i + 1) % ring_vertices.size()];
		v1.normal = v2.normal = z < 0 ? Vector3f(0, 0, -1) : Vector3f(0, 0, 1);
		result.insert(result.end(), { v0, v1, v2 });
	}
	return result;
}
