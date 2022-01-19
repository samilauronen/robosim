#include "BoxMesh.hpp"

using namespace Eigen;

BoxMesh::BoxMesh(float width, float height, float length, Vector3f color) :
	Mesh(),
	width_(width),
	height_(height),
	length_(length)
{
	std::vector<Quad> faces;

	faces.emplace_back(width, height, Vector3f(0, 0, 0), Vector3f(0, 0, -1), Vector3f(1, 0, 0));
	faces.emplace_back(width, height, Vector3f(0, 0, length), Vector3f(0, 0, 1), Vector3f(1, 0, 0));
	faces.emplace_back(width, length, Vector3f(0, height/2, length / 2), Vector3f(0, 1, 0), Vector3f(1, 0, 0));
	faces.emplace_back(width, length, Vector3f(0, -height / 2, length / 2), Vector3f(0, -1, 0), Vector3f(1, 0, 0));
	faces.emplace_back(height, length, Vector3f(-width/2, 0, length / 2), Vector3f(-1, 0,0), Vector3f(0, 1, 0));
	faces.emplace_back(height, length, Vector3f(width / 2, 0, length / 2), Vector3f(1, 0, 0), Vector3f(0, 1, 0));

	for (const Quad& face : faces) {
		std::vector<Vertex> face_verts = face.getVertices();
		vertices_.insert(vertices_.end(), face_verts.begin(), face_verts.end());
	}

	// set colors
	int i = 0;
	for (auto& v : vertices_) {
		v.color = color;
		i++;
	}
}

Quad::Quad(float width, float height, Vector3f center_point, Vector3f normal, Vector3f x)
{
	Vector3f y = normal.cross(x);
	Vector3f c = center_point;
	float xoff = width / 2;
	float yoff = height / 2;
	Vertex v1, v2, v3, v4;
	v1.position = c + x * xoff + y * yoff;
	v2.position = c - x * xoff - y * yoff;
	v3.position = c - x * xoff + y * yoff;
	v4.position = c + x * xoff - y * yoff;

	v1.normal = v2.normal = v3.normal = v4.normal = normal;
	vertices_.insert(vertices_.end(), { v1,v3,v2, v1,v2,v4 });  // CCW ordering
}
