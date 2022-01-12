#pragma once
#include "Mesh.hpp"

class BoxMesh : public Mesh {
public:
	BoxMesh(float width, float height, float length, FW::Vec3f color);
private:
	float width_, height_, length_;
};
