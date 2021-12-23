#include "RobotGraphics.hpp"
#include "Utility.hpp"

using namespace std;


void RobotGraphics::drawFrame(Mat4f world_to_frame, float scale) {
	Vec3f origin = world_to_frame * Vec3f(0, 0, 0);
	Mat3f orientation = world_to_frame.getXYZ();

	Vec3f i = orientation.getCol(0) * scale;
	Vec3f j = orientation.getCol(1) * scale;
	Vec3f k = orientation.getCol(2) * scale;

	glBegin(GL_LINES);
	glColor3f(1, 0, 0);
	glVertex3f(origin.x, origin.y, origin.z);
	glVertex3f(origin.x + i.x, origin.y + i.y, origin.z + i.z);
	glColor3f(0, 1, 0); // green
	glVertex3f(origin.x, origin.y, origin.z);
	glVertex3f(origin.x + j.x, origin.y + j.y, origin.z + j.z);
	glColor3f(0, 0, 1); // blue
	glVertex3f(origin.x, origin.y, origin.z);
	glVertex3f(origin.x + k.x, origin.y + k.y, origin.z + k.z);
	glEnd();
}

void RobotGraphics::renderSkeleton(vector<Link> links, Mat4f worldToBase)
{
	glEnable(GL_POINT_SMOOTH);
	glPointSize(15);

	// draw world origin frame:
	glLineWidth(2);
	float scale = 0.3;
	drawFrame(Mat4f{}, scale);

	// draw robot base frame
	scale = 0.1;
	drawFrame(worldToBase, scale);

	// draw link frames
	for (int i = 0; i < links.size(); i++) {
		Mat4f current_trans = links[i].to_world;

		bool is_tcp_frame = i + 1 == links.size();

		Vec3f origin = current_trans * Vec3f(0, 0, 0);

		// draw a point at the origin of the new frame
		// only draw if the current frame location is also a joint
		if (!is_tcp_frame) {
			glBegin(GL_POINTS);
			glColor3f(1.0f, 1.0f, 1.0f);
			glVertex3f(origin.x, origin.y, origin.z);
			glEnd();
		}

		// NOTE: disable this if it causes problems with any calculations in the future!
		// rotate frames with joint rotation, even though that doesnt follow DH standard?
		bool showJointRotation = false;
		Mat4f jointRotation;
		if (showJointRotation && !is_tcp_frame && (i-1) > 0 ) {
			Mat3f rot = Mat3f::rotation(Vec3f(0, 0, 1), links[i - 1].rotation);
			jointRotation.setCol(0, Vec4f(rot.getCol(0), 0));
			jointRotation.setCol(1, Vec4f(rot.getCol(1), 0));
			jointRotation.setCol(2, Vec4f(rot.getCol(2), 0));
		}
		// END NOTE ===================================================================

		// draw coordinate axes of the frame
		if (is_tcp_frame) {
			glLineWidth(2);
			scale = 0.1;
		}
		else {
			glLineWidth(1);
			scale = 0.075;
		}
		drawFrame(current_trans * jointRotation, scale);

		// draw links between frames
		glLineWidth(1);
		glBegin(GL_LINES);
		int parent = i - 1;
		Vec3f parent_world_pos, after_z_screw;
		glColor3f(1, 1, 1);
		if (parent != -1) {
			// may draw two lines, accounting for translation in z and x
			parent_world_pos = links[parent].to_world * Vec3f(0, 0, 0);
			after_z_screw = links[parent].to_world * links[i].z_screw * Vec3f(0, 0, 0);
		}
		else {
			parent_world_pos = worldToBase * Vec3f(0, 0, 0);
			after_z_screw = worldToBase * links[i].z_screw * Vec3f(0, 0, 0);
		}

		float z_screw_len = (after_z_screw - parent_world_pos).length();
		float x_screw_len = (origin - after_z_screw).length();

		// draw line if it is longer than some threshold,
		if (z_screw_len > 0.01) {
			glVertex3f(parent_world_pos.x, parent_world_pos.y, parent_world_pos.z);
			glVertex3f(after_z_screw.x, after_z_screw.y, after_z_screw.z);
		}
		if (x_screw_len > 0.01) {
			glVertex3f(after_z_screw.x, after_z_screw.y, after_z_screw.z);
			glVertex3f(origin.x, origin.y, origin.z);
		}
		
		glEnd();
	}
}

vector<Vertex> RobotGraphics::getMeshVertices(vector<Link> links, Mat4f world_to_base) {
	vector<Vertex> allVertices;

	float linkLength = 0;
	float init_link_thickness = 0.1f;
	float init_joint_radius = 0.07f;
	float init_joint_length = 0.2f;
	float size_reduction_factor = 0.8;

	// link meshes
	for (unsigned i = 0; i < links.size(); i++) {
		Mat4f parent_to_world = i == 0 ? world_to_base : links[i - 1].to_world;
		
		// position of current (i:th) frame
		Vec3f current_world_pos = links[i].to_world * Vec3f(0, 0, 0);

		// position of parent frame and the (non-existent) frame after only applying z-screw component of the next link matrix
		Vec3f parent_world_pos = parent_to_world * Vec3f(0, 0, 0);
		Vec3f after_z_screw = parent_to_world * links[i].z_screw * Vec3f(0, 0, 0);

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
			vector<Vertex> meshVertices = z_link.getVertices();

			Mat3f basisVecs = (parent_to_world * links[i].z_screw).getXYZ();

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
			vector<Vertex> meshVertices = x_link.getVertices();

			Mat3f basisVecs = links[i].to_world.getXYZ();
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
	}

	// joint meshes (size - 1 because last frame is not a joint, it's TCP frame)
	for (int i = -1; i < (int)links.size() - 1; i++) {
		float reduction = FW::pow(size_reduction_factor, i);
		float radius = init_joint_radius * reduction;
		float length = init_joint_length * reduction;

		// create mesh
		JointMesh joint(radius, length);
		vector<Vertex> meshVertices = joint.getVertices();

		Mat4f link_to_world = i == -1 ? world_to_base : links[i].to_world;

		// orient along the joint's z-axis 
		Mat3f orientation = link_to_world.getXYZ();

		// translate to world position of joint 
		Vec3f translation = link_to_world * Vec3f(0, 0, 0);

		// apply transformation to vertices
		Mat4f transformation = combineToMat4f(orientation, translation);
		for (auto& v : meshVertices) {
			v.position = transformation * v.position;
			v.normal = transformation.getXYZ().inverted().transposed() * v.normal;
		}
		allVertices.insert(allVertices.end(), meshVertices.begin(), meshVertices.end());
	}

	return allVertices;
}

std::vector<Vertex> RobotGraphics::LinkMesh::getVertices()
{
	float offset = thickness_ / 2.f;

	Vec3f normal = Vec3f(1, 0, 0);

	static const Vec3f distinct_colors[6] = {
		Vec3f(0, 0, 1), Vec3f(0, 1, 0), Vec3f(0, 1, 1),
		Vec3f(1, 0, 0), Vec3f(1, 0, 1), Vec3f(1, 1, 0) };

	vector<Vertex> vertices;

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

std::vector<Vertex> RobotGraphics::JointMesh::getVertices()
{
	vector<Vertex> allVertices;

	int numRingVerts = 32;

	// first face to z = -length/2
	auto face1 = makeFace(numRingVerts, -depth_ / 2);
	vector<Vertex> face1Tris = face1.first;
	vector<Vertex> face1Ring = face1.second;
	allVertices.insert(allVertices.end(), face1Tris.begin(), face1Tris.end());

	// second face to z = depth/2
	auto face2 = makeFace(numRingVerts, depth_ / 2);
	vector<Vertex> face2Tris = face2.first;
	vector<Vertex> face2Ring = face2.second;
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
std::pair<std::vector<Vertex>, std::vector<Vertex>> RobotGraphics::JointMesh::makeFace(int numRingVerts, float z)
{
	vector<Vertex> result;

	// middle vertex
	Vertex v0;
	v0.position = Vec3f(0, 0, z);
	v0.normal = v0.position.normalized();

	// generate ring of vertices around it
	// x = r cos t, y= r sin t
	vector<Vertex> ringVertices;
	for (float t = 0; t < 2 * FW_PI; t += 2 * FW_PI / numRingVerts) {
		Vertex v;
		v.position = Vec3f(radius_ * FW::cos(t), radius_ * FW::sin(t), z);
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


