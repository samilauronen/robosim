#include "BoxMesh.hpp"

BoxMesh::BoxMesh(float width, float height, float length, FW::Vec3f color):
	width_(width),
	height_(height),
	length_(length)
{
	using namespace FW;
	float xoff = width_ / 2.f;
	float yoff = height_ / 2.f;

	// first square face
	Vertex v1, v2, v3, v4;
	v1.position = Vec3f(-xoff, yoff, 0);
	v1.normal = Vec3f(-1, 1, -1).normalized();
	v2.position = Vec3f(xoff, yoff, 0);
	v2.normal = Vec3f(1, 1, -1).normalized();
	v3.position = Vec3f(xoff, -yoff, 0);
	v3.normal = Vec3f(1, -1, -1).normalized();
	v4.position = Vec3f(-xoff, -yoff, 0);
	v4.normal = Vec3f(-1, -1, -1).normalized();

	// triangles of the face: v1,v2,v3 and v3,v4,v1
	vertices_.insert(vertices_.end(), { v1,v2,v3, v3,v4,v1 });

	// second square face
	Vertex v5, v6, v7, v8;
	v5.position = Vec3f(-xoff, yoff, length_);
	v5.normal = Vec3f(-1, 1, 1).normalized();
	v6.position = Vec3f(xoff, yoff, length_);
	v6.normal = Vec3f(1, 1, 1).normalized();
	v7.position = Vec3f(xoff, -yoff, length_);
	v7.normal = Vec3f(1, -1, 1).normalized();
	v8.position = Vec3f(-xoff, -yoff, length_);
	v8.normal = Vec3f(-1, -1, 1).normalized();

	// triangles of the face: v5,v6,v7 and v7,v8,v5
	vertices_.insert(vertices_.end(), { v5,v6,v7, v7,v8,v5 });

	// triangles for faces along z-axis:
	vertices_.insert(vertices_.end(), { v5,v1,v4, v5,v8,v4 });
	vertices_.insert(vertices_.end(), { v5,v1,v2, v2,v5,v6 });
	vertices_.insert(vertices_.end(), { v3,v4,v7, v8,v7,v4 });
	vertices_.insert(vertices_.end(), { v6,v7,v2, v3,v2,v7 });

	// set colors
	int i = 0;
	for (auto& v : vertices_) {
		v.color = color;
		i++;
	}
}
