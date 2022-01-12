#pragma once
#include <vector>

#include "base/Math.hpp"

struct Vertex
{
	FW::Vec3f position;
	FW::Vec3f normal;
	FW::Vec3f color;
};

class Mesh {
public:
	void setToWorldTransform(FW::Mat4f t);
	void transform(FW::Mat4f t);

	std::vector<Vertex> getVertices() const;
protected:
	std::vector<Vertex> vertices_;
	FW::Mat4f to_world_;
};