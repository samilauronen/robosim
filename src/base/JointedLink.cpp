#include "JointedLink.hpp"
#include "utility.hpp"

// ============= LOGIC ===================

JointedLink::JointedLink(DhParam params, float joint_rotation) :
	params_(params),
	rotation_(joint_rotation),
	target_rotation_(joint_rotation),
	joint_speed_(0)
{
	this->updateLinkMatrix(joint_rotation);
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

void JointedLink::updateLinkMatrix(float rotationAngle) {
	link_matrix_ = this->evalLinkMatrix(rotationAngle);
}


// ============= GRAPHICS ====================

static const FW::Vec3f JOINT_COLOR = FW::Vec3f(0.05, 0.6, 0.4);
static const FW::Vec3f LINK_COLOR = FW::Vec3f(0.9, 0.5, 0);

// uses immediate mode to draw a simple "skeleton" model of joint-link combination
void JointedLink::renderSkeleton() const
{
	using namespace FW;

	Vec3f current_world_pos, parent_world_pos, after_z_screw;
	Mat4f parent_transform = to_world_ * link_matrix_.inverted();
	parent_world_pos = parent_transform * Vec3f(0, 0, 0);
	current_world_pos = to_world_ * Vec3f(0, 0, 0);
	after_z_screw = parent_transform * Vec3f(0, 0, params_.d);	// moving by offset towards prev frame's z

	// draw a point at the origin of the previous frame
	glBegin(GL_POINTS);
	glColor3f(1.0f, 1.0f, 1.0f);
	glVertex3f(parent_world_pos.x, parent_world_pos.y, parent_world_pos.z);
	glEnd();

	// NOTE: disable this if it causes problems with any calculations in the future!
	// rotate frames with joint rotation, even though that doesnt follow DH standard?
	//bool showJointRotation = false;
	Mat4f jointRotation;
	//if (showJointRotation && !is_tcp_frame && (i - 1) > 0) {
	//	Mat3f rot = Mat3f::rotation(Vec3f(0, 0, 1), links[i - 1].rotation);
	//	jointRotation.setCol(0, Vec4f(rot.getCol(0), 0));
	//	jointRotation.setCol(1, Vec4f(rot.getCol(1), 0));
	//	jointRotation.setCol(2, Vec4f(rot.getCol(2), 0));
	//}
	// END NOTE ===================================================================

	// draw coordinate axes of the frame
	glLineWidth(1);
	float scale = 0.075;
	drawFrame(parent_transform * jointRotation, scale);

	// draw link between current frame and previous frame
	glLineWidth(1);
	glBegin(GL_LINES);

	glColor3f(1, 1, 1);

	float z_screw_len = (after_z_screw - parent_world_pos).length();
	float x_screw_len = (current_world_pos - after_z_screw).length();

	// draw line if it is longer than some threshold,
	if (z_screw_len > 0.01) {
		glVertex3f(parent_world_pos.x, parent_world_pos.y, parent_world_pos.z);
		glVertex3f(after_z_screw.x, after_z_screw.y, after_z_screw.z);
	}
	if (x_screw_len > 0.01) {
		glVertex3f(after_z_screw.x, after_z_screw.y, after_z_screw.z);
		glVertex3f(current_world_pos.x, current_world_pos.y, current_world_pos.z);
	}

	glEnd();
}

std::vector<Vertex> JointedLink::getMeshVertices(int i) const{
	using namespace FW;
	std::vector<Vertex> allVertices;

	float linkLength = 0;
	float init_link_thickness = 0.1f;
	float init_joint_radius = 0.07f;
	float init_joint_length = 0.2f;
	float size_reduction_factor = 0.8;

	// link meshes
	Mat4f parent_to_world = to_world_ * link_matrix_.inverted();

	// position of current (i:th) frame
	Vec3f current_world_pos = to_world_ * Vec3f(0, 0, 0);

	// position of parent frame and the (non-existent) frame after only applying z-screw component of the next link matrix
	Vec3f parent_world_pos = parent_to_world * Vec3f(0, 0, 0);
	Vec3f after_z_screw = parent_to_world * Vec3f(0, 0, params_.d);

	// lengths of the possible links
	float z_link_len = (after_z_screw - parent_world_pos).length();
	float x_link_len = (current_world_pos - after_z_screw).length();

	bool bothLinks = z_link_len > 0.01 && x_link_len > 0.01;

	// create link mesh if length is over some threshold
	if (z_link_len > 0.01) {
		if (bothLinks) {
			z_link_len += init_link_thickness * FW::pow(size_reduction_factor, i) / 2;
		}
		LinkMesh z_link(init_link_thickness * FW::pow(size_reduction_factor, i), z_link_len);
		std::vector<Vertex> meshVertices = z_link.getVertices();

		Mat3f basisVecs = (parent_to_world * combineToMat4f(Mat3f::rotation(Vec3f(0,0,1), rotation_), Vec3f(0,0,0))).getXYZ();

		Vec3f i, j, k;
		k = (after_z_screw - parent_world_pos).normalized();
		j = basisVecs.getCol(1).normalized();
		i = j.cross(k);

		Mat3f link_orientation;
		link_orientation.setCol(0, i);
		link_orientation.setCol(1, j);
		link_orientation.setCol(2, k);

		Vec3f link_translation = parent_world_pos;

		Mat4f link_transform = combineToMat4f(link_orientation, link_translation);

		for (auto& v : meshVertices) {
			v.position = link_transform * v.position;
			v.normal = link_transform.getXYZ() * v.normal;
		}
		allVertices.insert(allVertices.end(), meshVertices.begin(), meshVertices.end());
	}

	if (x_link_len > 0.01) {
		if (bothLinks) {
			x_link_len -= init_link_thickness * FW::pow(size_reduction_factor, i) / 2;
		}
		LinkMesh x_link(init_link_thickness * FW::pow(size_reduction_factor, i), x_link_len);
		std::vector<Vertex> meshVertices = x_link.getVertices();

		Mat3f basisVecs = to_world_.getXYZ();
		Vec3f i, j, k;
		k = (after_z_screw - current_world_pos).normalized();
		j = basisVecs.getCol(1).normalized();
		i = j.cross(k);

		Mat3f link_orientation;
		link_orientation.setCol(0, i);
		link_orientation.setCol(1, j);
		link_orientation.setCol(2, k);

		Vec3f link_translation = current_world_pos;

		Mat4f link_transform = combineToMat4f(link_orientation, link_translation);

		for (auto& v : meshVertices) {
			v.position = link_transform * v.position;
			v.normal = link_transform.getXYZ() * v.normal;
		}
		allVertices.insert(allVertices.end(), meshVertices.begin(), meshVertices.end());
	}

	// joint mesh
	float reduction = FW::pow(size_reduction_factor, i);
	float radius = init_joint_radius * reduction;
	float length = init_joint_length * reduction;

	// create mesh
	JointMesh joint(radius, length);
	std::vector<Vertex> meshVertices = joint.getVertices();

	// joint is placed at the parent frame's location and oriented along its z-axiz
	Mat4f joint_to_world = to_world_ * link_matrix_.inverted();

	// orient along the joint's z-axis 
	Mat3f orientation = joint_to_world.getXYZ();

	// translate to world position of joint 
	Vec3f translation = joint_to_world * Vec3f(0, 0, 0);

	// apply transformation to vertices
	Mat4f transformation = combineToMat4f(orientation, translation);
	for (auto& v : meshVertices) {
		v.position = transformation * v.position;
		v.normal = transformation.getXYZ().inverted().transposed() * v.normal;
	}
	allVertices.insert(allVertices.end(), meshVertices.begin(), meshVertices.end());

	return allVertices;
}

std::vector<Vertex> LinkMesh::getVertices()
{
	using namespace FW;

	float offset = thickness_ / 2.f;

	Vec3f normal = Vec3f(1, 0, 0);

	std::vector<Vertex> vertices;

	// first square face
	Vertex v1, v2, v3, v4;
	v1.position = Vec3f(-offset, offset, 0);
	v1.normal = Vec3f(-1, 1, -1).normalized();
	v2.position = Vec3f(offset, offset, 0);
	v2.normal = Vec3f(1, 1, -1).normalized();
	v3.position = Vec3f(offset, -offset, 0);
	v3.normal = Vec3f(1, -1, -1).normalized();
	v4.position = Vec3f(-offset, -offset, 0);
	v4.normal = Vec3f(-1, -1, -1).normalized();

	// triangles of the face: v1,v2,v3 and v3,v4,v1
	vertices.insert(vertices.end(), { v1,v2,v3, v3,v4,v1 });

	// second square face
	Vertex v5, v6, v7, v8;
	v5.position = Vec3f(-offset, offset, length_);
	v5.normal = Vec3f(-1, 1, 1).normalized();
	v6.position = Vec3f(offset, offset, length_);
	v6.normal = Vec3f(1, 1, 1).normalized();
	v7.position = Vec3f(offset, -offset, length_);
	v7.normal = Vec3f(1, -1, 1).normalized();
	v8.position = Vec3f(-offset, -offset, length_);
	v8.normal = Vec3f(-1, -1, 1).normalized();

	// triangles of the face: v5,v6,v7 and v7,v8,v5
	vertices.insert(vertices.end(), { v5,v6,v7, v7,v8,v5 });

	// triangles for faces along z-axis:
	vertices.insert(vertices.end(), { v5,v1,v4, v5,v8,v4 });
	vertices.insert(vertices.end(), { v5,v1,v2, v2,v5,v6 });
	vertices.insert(vertices.end(), { v3,v4,v7, v8,v7,v4 });
	vertices.insert(vertices.end(), { v6,v7,v2, v3,v2,v7 });

	// set colors
	int i = 0;
	for (auto& v : vertices) {
		v.color = LINK_COLOR;
		i++;
	}

	return vertices;
}

std::vector<Vertex> JointMesh::getVertices()
{
	std::vector<Vertex> allVertices;

	int numRingVerts = 32;

	// first face to z = -length/2
	auto face1 = makeFace(numRingVerts, -depth_ / 2);
	std::vector<Vertex> face1Tris = face1.first;
	std::vector<Vertex> face1Ring = face1.second;
	allVertices.insert(allVertices.end(), face1Tris.begin(), face1Tris.end());

	// second face to z = depth/2
	auto face2 = makeFace(numRingVerts, depth_ / 2);
	std::vector<Vertex> face2Tris = face2.first;
	std::vector<Vertex> face2Ring = face2.second;
	allVertices.insert(allVertices.end(), face2Tris.begin(), face2Tris.end());

	// now make the lateral surface
	for (int i = 0; i < numRingVerts; i++) {
		// make two triangles in the reactangle formed by two pairs of vertices on the opposite faces
		Vertex v1, v2, v3, v4;

		v1 = face1Ring[i];
		v2 = face1Ring[(i + 1) % numRingVerts];

		v3 = face2Ring[i];
		v4 = face2Ring[(i + 1) % numRingVerts];

		allVertices.insert(allVertices.end(), { v1,v2,v4 ,v1,v3,v4 });
	}

	// same color for all
	for (auto& v : allVertices) {
		v.color = JOINT_COLOR;
	}

	return allVertices;
}

// returns pair of triangle vertices, and ring vertices
std::pair<std::vector<Vertex>, std::vector<Vertex>> JointMesh::makeFace(int numRingVerts, float z)
{
	std::vector<Vertex> result;

	// middle vertex
	Vertex v0;
	v0.position = FW::Vec3f(0, 0, z);
	v0.normal = v0.position.normalized();

	// generate ring of vertices around it
	// x = r cos t, y= r sin t
	std::vector<Vertex> ringVertices;
	for (float t = 0; t < 2 * FW_PI; t += 2 * FW_PI / numRingVerts) {
		Vertex v;
		v.position = FW::Vec3f(radius_ * FW::cos(t), radius_ * FW::sin(t), z);
		v.normal = v.position.normalized();
		//v.normal.z = 1;
		//v.normal.normalize();
		ringVertices.push_back(v);
	}

	// for each pair of ring vertices, create triangle from it to the center vertex
	for (int i = 0; i < numRingVerts; i += 1) {
		Vertex v1 = ringVertices[i];
		Vertex v2 = ringVertices[(i + 1) % numRingVerts];
		result.insert(result.end(), { v0, v1, v2 });
	}

	return { result, ringVertices };
}







