#include <assert.h>

#include "JointedLink.hpp"
#include "Utility.hpp"

using namespace Eigen;

JointedLink::JointedLink(DhParam params, float joint_rotation, int link_number) :
	mesh_(params.d, params.a, link_number),
	params_(params),
	rotation_(joint_rotation),
	target_rotation_(joint_rotation),
	link_number_(link_number),
	joint_speed_(0),
	to_world_(Affine3f::Identity())
{
	this->updateLinkMatrix(rotation_);
}

void JointedLink::update(float dt, Affine3f current_world_transform)
{
	rotation_ += joint_speed_ * dt;
	updateLinkMatrix(rotation_);
	setToWorld(current_world_transform * link_matrix_);
	updateMesh();
}

void JointedLink::updateLinkMatrix(float rotationAngle) {
	link_matrix_ = this->evalLinkMatrix(rotationAngle);
}

void JointedLink::updateMesh() {
	// the mesh is oriented using the joint's world transform
	Affine3f joint_to_world = to_world_ * link_matrix_.inverse();

	// align with joint's z axis and rotate around it by the joint's rotation
	// NOTE: this differs from DH standard somewhat, since this would imply that the
	// frame that exists at the joint's location would be rotated along with the joint (in DH stadard it is stationary)
	Matrix3f joint_orientation = joint_to_world.linear().matrix();
	joint_orientation *= AngleAxisf(rotation_, Vector3f::UnitZ()).toRotationMatrix();

	// translate to the world position of the joint 

	// apply
	Affine3f transformation;
	transformation.linear() = joint_orientation;
	transformation.translation() = joint_to_world.translation();
	mesh_.setToWorldTransform(transformation);
}

void JointedLink::recreateMesh() {
	mesh_ = JointedLinkMesh(params_.d, params_.a, link_number_);
}

Affine3f JointedLink::evalLinkMatrix(float rotationAngle) const
{
	// https://www.youtube.com/watch?v=nuB_7BkYNMk
	// first move up so that we are at the level of the common normal
	// then rotate around z to align x with next x
	Affine3f z_screw =
		AngleAxisf(rotationAngle, Vector3f::UnitZ()) *
		Translation3f(0, 0, params_.d);

	// after applying the z-screw, we are at the correct level and our x-axis points in the correct direction
	// now move towards our new x by link length and apply link twist (rotate around x-axis)
	Affine3f x_screw =
		AngleAxisf(params_.alpha, Vector3f::UnitX()) *
		Translation3f(params_.a, 0, 0);

	// combine screws
	return z_screw * x_screw;
}

// uses immediate mode to draw a simple "skeleton" model of the joint-link combination
void JointedLink::renderSkeleton() const
{
	Vector3f current_world_pos, parent_world_pos, after_z_screw;
	Affine3f parent_transform = to_world_ * link_matrix_.inverse();
	parent_world_pos = parent_transform.translation();
	current_world_pos = to_world_.translation();
	after_z_screw = parent_transform * Vector3f(0, 0, params_.d);	// moving by offset towards prev frame's z
	float z_screw_len = (after_z_screw - parent_world_pos).norm();
	float x_screw_len = (current_world_pos - after_z_screw).norm();

	// draw a point at the origin of the previous frame
	glBegin(GL_POINTS);
	glColor3f(1.0f, 1.0f, 1.0f);
	glVertex3f(parent_world_pos.x(), parent_world_pos.y(), parent_world_pos.z());
	glEnd();

	// draw coordinate axes of the frame
	glLineWidth(1);
	float scale = 0.075;
	drawFrame(parent_transform, scale);

	// draw link lines
	glLineWidth(1);
	glBegin(GL_LINES);
	glColor3f(1, 1, 1);
	glVertex3f(parent_world_pos.x(), parent_world_pos.y(), parent_world_pos.z());
	glVertex3f(after_z_screw.x(), after_z_screw.y(), after_z_screw.z());
	glVertex3f(after_z_screw.x(), after_z_screw.y(), after_z_screw.z());
	glVertex3f(current_world_pos.x(), current_world_pos.y(), current_world_pos.z());
	glEnd();
}

std::vector<Vertex> JointedLink::getMeshVertices() const {
	return mesh_.getVertices();
}
