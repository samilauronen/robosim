#pragma once
#include "Mesh.hpp"

class BoxMesh : public Mesh {
public:
	BoxMesh(float width, float height, float length, Eigen::Vector3f color);
private:
	
	float width_, height_, length_;
};

class RectangleMesh : public Mesh {
public:
	RectangleMesh(float width, float height, Eigen::Vector3f center_point, Eigen::Vector3f normal, Eigen::Vector3f x);
};
