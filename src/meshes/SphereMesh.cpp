#define _USE_MATH_DEFINES
#include <math.h>

#include "SphereMesh.hpp"

using namespace Eigen;

SphereMesh::SphereMesh(float radius, Vector4f color, int angle_subdivisions) :
	Mesh(),
	color_(color),
	radius_(radius)
{
	vertices_.clear();

	float theta_increment = M_PI / angle_subdivisions;
	float phi_increment = M_PI / angle_subdivisions;

	Vertex first_point;
	first_point.position = Vector3f(0, 0, radius);
	first_point.normal = Vector3f(0, 0, 1);
	first_point.color = color_;

	Vertex last_point;
	last_point.position = Vector3f(0, 0, -radius);
	last_point.normal = Vector3f(0, 0, -1);
	last_point.color = color_;

	std::vector<Vertex> curr_ring_verts;
	std::vector<Vertex> prev_ring_verts;

	bool first_run = true;

	for (float theta = theta_increment; theta < M_PI - theta_increment; theta += theta_increment) {
		curr_ring_verts.clear();
		for (float phi = 0; phi < 2 * M_PI; phi += phi_increment) {
			// conversion to cartesian coordinates
			float x = radius * sin(theta) * cos(phi);
			float y = radius * sin(theta) * sin(phi);
			float z = radius * cos(theta);

			Vertex v;
			v.position = Vector3f(x, y, z);
			v.normal = Vector3f(x, y, z).normalized();
			v.color = color_;

			curr_ring_verts.push_back(v);
		}
		if (first_run)
		{
			first_run = false;
			std::vector<Vertex> first_ring_triangles = createTrianglesToVertex(curr_ring_verts, first_point);
			vertices_.insert(vertices_.end(), first_ring_triangles.begin(), first_ring_triangles.end());
		}
		else
		{
			std::vector<Vertex> ring_triangles = createTrianglesBetweenRings(prev_ring_verts, curr_ring_verts);
			vertices_.insert(vertices_.end(), ring_triangles.begin(), ring_triangles.end());
		}
		prev_ring_verts = curr_ring_verts;
	}

	std::vector<Vertex> last_ring_triangles = createTrianglesToVertex(prev_ring_verts, last_point);
	vertices_.insert(vertices_.end(), last_ring_triangles.begin(), last_ring_triangles.end());
}


std::vector<Vertex> SphereMesh::createTrianglesToVertex(const std::vector<Vertex>& ring_vertices, const Vertex center) const
{
	std::vector<Vertex> result;

	// for each pair of ring vertices, create triangle from it to the center vertex
	for (int i = 0; i < ring_vertices.size(); i += 1) {
		Vertex v1 = ring_vertices[i];
		Vertex v2 = ring_vertices[(i + 1) % ring_vertices.size()];
		result.insert(result.end(), { center, v1, v2 });
	}
	return result;
}

std::vector<Vertex> SphereMesh::createTrianglesBetweenRings(const std::vector<Vertex>& ring1_vertices, const std::vector<Vertex>& ring2_vertices) const
{
	std::vector<Vertex> result;
	assert(ring1_vertices.size() == ring2_vertices.size());

	int N = ring1_vertices.size();

	// create triangles between pairs of vertices
	for (int i = 0; i < N; i += 1) {
		Vertex v1 = ring1_vertices[i];
		Vertex v2 = ring2_vertices[i];
		Vertex v3 = ring2_vertices[(i + 1) % N];
		result.insert(result.end(), { v1, v2, v3 });

		v1 = v1;
		v2 = v3;
		v3 = ring1_vertices[(i + 1) % N];
		result.insert(result.end(), { v1, v2, v3 });
	}
	return result;
}
