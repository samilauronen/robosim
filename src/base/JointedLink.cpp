#include <assert.h>

#include "JointedLink.hpp"
#include "utility.hpp"


JointedLink::JointedLink(DhParam params, float joint_rotation, int link_number) :
	mesh_(params.d, params.a, link_number),
	params_(params),
	rotation_(joint_rotation),
	target_rotation_(joint_rotation),
	link_number_(link_number),
	joint_speed_(0)
{
	this->updateLinkMatrix(rotation_);
}

void JointedLink::update(float dt_millis, FW::Mat4f current_world_transform)
{
	rotation_ += joint_speed_ * (dt_millis / 1000);
	updateLinkMatrix(rotation_);
	setToWorld(current_world_transform * link_matrix_);
	updateMesh();
}

void JointedLink::updateLinkMatrix(float rotationAngle) {
	link_matrix_ = this->evalLinkMatrix(rotationAngle);
}

void JointedLink::updateMesh() {
	// the mesh is oriented using the joint's world transform
	FW::Mat4f joint_to_world = to_world_ * link_matrix_.inverted();

	// align with joint's z axis and rotate around it by the joint's rotation
	// NOTE: this differs from DH standard somewhat, since this would imply that the
	// frame that exists at the joint's location would be rotated along with the joint (in DH stadard it is stationary)
	FW::Mat3f orientation = joint_to_world.getXYZ() * FW::Mat3f::rotation(FW::Vec3f(0, 0, 1), rotation_);

	// translate to the world position of the joint 
	FW::Vec3f translation = joint_to_world * FW::Vec3f(0, 0, 0);

	// apply
	FW::Mat4f transformation = combineToMat4f(orientation, translation);
	mesh_.setToWorldTransform(transformation);
}

FW::Mat4f JointedLink::evalLinkMatrix(float rotationAngle) const
{
	using namespace FW;

	// https://www.youtube.com/watch?v=nuB_7BkYNMk
	Vec3f transl = Vec3f(0, 0, params_.d);						// first move up so that we are at the level of the common normal
	Mat3f rot = Mat3f::rotation(Vec3f(0, 0, 1), rotationAngle);	// then rotate around z to align x with next x

	Mat4f z_screw = combineToMat4f(rot, transl);  // translation and rotation with respect to z

	// after applying the z-screw, we are at the correct level and our x-axis points in the correct direction
	// now move towards our new x by link length
	transl = Vec3f(params_.a, 0, 0);
	rot = Mat3f::rotation(Vec3f(1, 0, 0), params_.alpha);	// apply link twist (rotate around x-axis)

	Mat4f x_screw = combineToMat4f(rot, transl);	// translation and rotation with respect to x

	return z_screw * x_screw;
}

// uses immediate mode to draw a simple "skeleton" model of the joint-link combination
void JointedLink::renderSkeleton() const
{
	using namespace FW;

	Vec3f current_world_pos, parent_world_pos, after_z_screw;
	Mat4f parent_transform = to_world_ * link_matrix_.inverted();
	parent_world_pos = parent_transform * Vec3f(0, 0, 0);
	current_world_pos = to_world_ * Vec3f(0, 0, 0);
	after_z_screw = parent_transform * Vec3f(0, 0, params_.d);	// moving by offset towards prev frame's z
	float z_screw_len = (after_z_screw - parent_world_pos).length();
	float x_screw_len = (current_world_pos - after_z_screw).length();

	// draw a point at the origin of the previous frame
	glBegin(GL_POINTS);
	glColor3f(1.0f, 1.0f, 1.0f);
	glVertex3f(parent_world_pos.x, parent_world_pos.y, parent_world_pos.z);
	glEnd();

	// draw coordinate axes of the frame
	glLineWidth(1);
	float scale = 0.075;
	drawFrame(parent_transform, scale);

	// draw link lines
	glLineWidth(1);
	glBegin(GL_LINES);
	glColor3f(1, 1, 1);
	glVertex3f(parent_world_pos.x, parent_world_pos.y, parent_world_pos.z);
	glVertex3f(after_z_screw.x, after_z_screw.y, after_z_screw.z);
	glVertex3f(after_z_screw.x, after_z_screw.y, after_z_screw.z);
	glVertex3f(current_world_pos.x, current_world_pos.y, current_world_pos.z);
	glEnd();
}

std::vector<Vertex> JointedLink::getMeshVertices() const {
	return mesh_.getVertices();
}
