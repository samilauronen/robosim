#include "robot.hpp"
#include "utility.hpp"

#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

Robot::Robot(std::string dh_param_filename, FW::Vec3f location)
	: selected_joint_(0),
	filename_(dh_param_filename)
{
	// base to 0 frame
	Mat3f rotation = Mat3f::rotation(Vec3f(1, 0, 0), FW_PI / 2);
	baseToZero_ = combineToMat4f(rotation, Vec3f(0, 0, 0));

	loadDhParams();
	buildModel();

	// set the transformation from base to world, no rotation
	baseToWorld_ = combineToMat4f(Mat3f(), location);

}

void Robot::renderSkeleton()
{
	glEnable(GL_POINT_SMOOTH);
	glPointSize(15);

	// Let's fetch the transforms you generated in Skeleton::updateToWorldTransforms().
	vector<Mat4f> transforms = getToWorldTransforms();

	// And loop through all the joints.
	for (auto i = 0u; i < transforms.size(); ++i) {
		// YOUR CODE HERE (R1)
		// Use the transforms to obtain the position of the joint in world space.
		// This should be a one-liner.
		Vec3f joint_world_pos = transforms[i] * Vec3f(0, 0, 0);

		// glBegin()-glEnd() with glVertex() commands in between is how draw calls
		// are done in immediate mode OpenGL.
		glBegin(GL_POINTS);
		if (i == (int)selected_joint_)
			glColor3f(1.0f, 0.2f, 0.2f);
		else
			glColor3f(1.0f, 1.0f, 1.0f);

		// Immediate mode command drawing an individual vertex
		glVertex3f(joint_world_pos.x, joint_world_pos.y, joint_world_pos.z);
		glEnd(); // we're done drawing points

		// YOUR CODE HERE (R3)
		// Use the transforms to obtain the joint's orientation.
		// (If you understand transformation matrices correctly, you can directly
		// read the these vectors off of the matrices.)
		Vec3f right, up, ahead;
		// Then let's draw some lines to show the joint coordinate system.
		// Draw a small coloured line segment from the joint's world position towards
		// each of its local coordinate axes (the line length should be determined by "scale").
		// The colors for each axis are already set for you below.
		float scale = 0.1;	// length for the coordinate system axes.
		glBegin(GL_LINES);

		Vec3f o = Vec3f(0, 0, 0);

		glColor3f(1, 0, 0); // red
		glVertex3f(o.x, o.y, o.z);
		glVertex3f(o.x + scale, o.y, o.z);
		glColor3f(0, 1, 0); // green
		glVertex3f(o.x, o.y, o.z);
		glVertex3f(o.x, o.y + scale, o.z);
		glColor3f(0, 0, 1); // blue
		glVertex3f(o.x, o.y, o.z);
		glVertex3f(o.x, o.y, o.z + scale);

		scale = 0.05;	// length for the coordinate system axes.

		// draw the x axis... ("right")
		glColor3f(1, 0, 0); // red
		Mat3f rotation = transforms[i].getXYZ();
		Vec3f new_i = rotation.getCol(0).normalized() * scale;
		Vec3f new_j = rotation.getCol(1).normalized() * scale;
		Vec3f new_k = rotation.getCol(2).normalized() * scale;
		glVertex3f(joint_world_pos.x, joint_world_pos.y, joint_world_pos.z);
		glVertex3f(joint_world_pos.x + new_i.x, joint_world_pos.y + new_i.y, joint_world_pos.z + new_i.z);

		// ..and the y axis.. ("up")
		glColor3f(0, 1, 0); // green
		glVertex3f(joint_world_pos.x, joint_world_pos.y, joint_world_pos.z);
		glVertex3f(joint_world_pos.x + new_j.x, joint_world_pos.y + new_j.y, joint_world_pos.z + new_j.z);

		// ..and the z axis ("ahead").
		glColor3f(0, 0, 1); // blue
		glVertex3f(joint_world_pos.x, joint_world_pos.y, joint_world_pos.z);
		glVertex3f(joint_world_pos.x + new_k.x, joint_world_pos.y + new_k.y, joint_world_pos.z + new_k.z);

		// Finally, draw a line segment from the world position of this joint to the world
		// position of the parent joint. You should first check if the parent exists
		// using skel_.getJointParent(i) - it returns -1 for the root, which has no parent.
		int parent = i - 1;
		if (parent != -1) {
			glColor3f(1, 1, 1);
			glVertex3f(joint_world_pos.x, joint_world_pos.y, joint_world_pos.z);
			Vec3f parent_world_pos = transforms[parent] * Vec3f(0, 0, 0);
			glVertex3f(parent_world_pos.x, parent_world_pos.y, parent_world_pos.z);
		}
		// ...
		glEnd(); // we're done drawing lines	
	}
}

vector<Vertex> Robot::getMeshVertices() 
{
	vector<Mat4f> transforms = getToWorldTransforms();
	vector<Vertex> allVertices;

	float linkLength = 0;
	bool isOffset = false;
	float init_link_thickness = 0.1f;
	float init_joint_radius = 0.07f;
	float init_joint_length = 0.2f;
	float size_reduction_factor = 0.8;

	// link meshes
	for (unsigned i = 0; i < transforms.size(); i++) {
		isOffset = !(FW::abs(joints_[i].p.a) > 0.00001);  // links use either offset (d) or link length (a). Both are not allowed by this implementation

		// create link meshes
		linkLength = FW::abs(isOffset ? joints_[i].p.d : joints_[i].p.a);
		LinkMesh link(init_link_thickness * FW::pow(size_reduction_factor, i), linkLength);

		vector<Vertex> meshVertices = link.getVertices();

		// translate and rotate link meshes to correct position
		Vec3f joint_world_pos = transforms[i] * Vec3f(0, 0, 0);
		int parent = i - 1;
		if (parent != -1) {
			Vec3f parent_world_pos = transforms[parent] * Vec3f(0, 0, 0);
			Vec3f norm_diff = (parent_world_pos - joint_world_pos).normalized();

			Mat3f basisVecs = transforms[i].getXYZ();
			Vec3f j;
			if (isOffset) {
				j = basisVecs.getCol(1).normalized();
			}
			else {
				j = basisVecs.getCol(2).normalized();
			}
			Vec3f i = j.cross(norm_diff);
			Mat3f orientation;
			orientation.setCol(0, i);
			orientation.setCol(1, j);
			orientation.setCol(2, norm_diff);

			Vec3f translation = joint_world_pos;
			Mat4f transform = combineToMat4f(orientation, translation);

			for (auto& v : meshVertices) {
				v.position = transform * v.position;
				v.normal = transform * v.normal;
			}
		}
		allVertices.insert(allVertices.end(), meshVertices.begin(), meshVertices.end());
	}

	// joint meshes
	for (unsigned i = 0; i < transforms.size(); i++) {
		float reduction = FW::pow(size_reduction_factor, i);
		float radius = init_joint_radius * reduction;
		float length = init_joint_length * reduction;

		// create mesh
		JointMesh joint(radius, length);
		vector<Vertex> meshVertices = joint.getVertices();

		// orient along the joint's z-axis
		Mat3f jointBasisVecs = transforms[i].getXYZ();
		Mat3f orientation = jointBasisVecs;

		// translate to world position of joint 
		Vec3f translation = transforms[i] * Vec3f(0, 0, 0);
		//translation.z += length / 2;	// center the mesh

		// create transformation matrix and transform vertices with it
		Mat4f transformation = combineToMat4f(orientation, translation);
		for (auto& v : meshVertices) {
			v.position = transformation * v.position;
			v.normal = transformation * v.normal;
		}
		allVertices.insert(allVertices.end(), meshVertices.begin(), meshVertices.end());
	}

	return allVertices;
}

void Robot::updateToWorldTransforms()
{
	Mat4f prev_to_world = baseToWorld_;
	for (int i = 0; i < joints_.size(); i++) {
		Joint& j = joints_[i];

		// update to parent, based on changed rotation
		Vec3f translation = -Vec3f(0, 0, j.p.d);						// first move up so that we are at the level of the common normal
		Mat3f rotation = Mat3f::rotation(Vec3f(0, 0, 1), j.rotation);	// then rotate around z to align x with next x. NOTE: this should be done with some constant value, instead of the variable angle? Fix later.

		Mat4f z_screw = combineToMat4f(rotation, translation);		// translation and rotation with respect to z

		//cerr << j.rotation << endl;

		// after applying the z-screw, we are at the correct level and our x-axis points in the correct direction
		// now move towards our new x by link length
		translation = -Vec3f(j.p.a, 0, 0);
		Mat3f rot2 = Mat3f::rotation(Vec3f(1, 0, 0), j.p.alpha);
		Mat4f x_screw = combineToMat4f(rot2, translation);

		// our final transform is now application of screw to z and screw to x in that order
		Mat4f tf = z_screw * x_screw;
		j.to_parent = tf.inverted();

		j.to_world = prev_to_world * j.to_parent;
		//cerr << "To world for joint: " << i << endl;
		//j.to_world.print(); cerr << endl << endl;
		prev_to_world = j.to_world;
	}
}

const std::vector<Joint>& Robot::getJoints()
{
	updateToWorldTransforms();
	return joints_;
}

std::vector<FW::Mat4f> Robot::getToWorldTransforms()
{
	updateToWorldTransforms();
	vector<Mat4f> transforms;
	for (const auto& j : joints_)
		transforms.push_back(j.to_world);
	return transforms;
}

void Robot::loadDhParams()
{
	ifstream file_input(filename_, ios::in);
	string line;

	while (getline(file_input, line)) {
		if (line[0] == '#') continue;

		stringstream ss(line);
		DhParam p;
		string sink;

		ss >> p.sigma;

		if (p.sigma == "R") {
			ss >> p.a >> p.d >> p.alpha >> sink;
			p.theta = -1;
		} else if (p.sigma == "P") {
			ss >> p.a >> sink >> p.alpha >> p.theta;
			p.d = -1;
		}
		else {
			cerr << "Invalid joint type: " << p.sigma << endl;
			continue;
		}
		
		params_.push_back(p);
	}
}

void Robot::buildModel()
{
	// joint 1 -- between link 0 and link 1
	Joint j1;
	j1.p = DhParam{};
	j1.rotation = 0;
	j1.position = Vec3f(0, 0, 0);
	j1.to_parent = Mat4f();
	joints_.push_back(j1);

	int i = 0;

	for (DhParam p : params_) {

		cerr << "Transformations for " << i << ":th joint:" << endl;

		// https://www.youtube.com/watch?v=nuB_7BkYNMk
		Vec3f translation = -Vec3f(0, 0, p.d);						// first move up so that we are at the level of the common normal
		Mat3f rotation = Mat3f::rotation(Vec3f(0, 0, 1), 0);	// then rotate around z to align x with next x. NOTE: this should be done with some constant value, instead of the variable angle? Fix later.

		Mat4f z_screw = combineToMat4f(rotation, translation);		// translation and rotation with respect to z

		cerr << "z screw:" << endl;
		z_screw.print(); cerr << endl;

		// after applying the z-screw, we are at the correct level and our x-axis points in the correct direction
		// now move towards our new x by link length
		translation = -Vec3f(p.a, 0, 0);
		Mat3f rot2 = Mat3f::rotation(Vec3f(1, 0, 0), p.alpha);

		Mat4f x_screw = combineToMat4f(rot2, translation);

		cerr << "x screw:" << endl;
		x_screw.print(); cerr << endl;

		// our final transform is now application of screw to z and screw to x in that order
		Mat4f tf = z_screw * x_screw;
		cerr << "tf" << endl;
		tf.print(); cerr << endl;

		cerr << "Trasformation to prev joint from " << i << ":th joint:" << endl;
		tf.inverted().print(); cerr << endl;
		cerr << endl;

		Joint j;
		j.p = p;
		j.rotation = 0;
		j.position = Vec3f(0, 0, 0);
		j.to_parent = tf.inverted();
		joints_.push_back(j);
		i++;
	}
}

std::vector<Vertex> LinkMesh::getVertices()
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
	v1.normal = Vec3f(-1, 1, 0).normalized();
	v2.position = Vec3f(offset, offset, 0);
	v2.normal = Vec3f(1, 1, 0).normalized();
	v3.position = Vec3f(offset, -offset, 0);
	v3.normal = Vec3f(1, -1, 0).normalized();
	v4.position = Vec3f(-offset, -offset, 0);
	v4.normal = Vec3f(-1, -1, 0).normalized();

	// triangles of the face: v1,v2,v3 and v3,v4,v1
	vertices.insert(vertices.end(), { v1,v2,v3, v3,v4,v1 });

	// second square face
	Vertex v5, v6, v7, v8;
	v5.position = Vec3f(-offset, offset, length_);
	v5.normal = Vec3f(-1, 1, 0).normalized();
	v6.position = Vec3f(offset, offset, length_);
	v6.normal = Vec3f(1, 1, 0).normalized();
	v7.position = Vec3f(offset, -offset, length_);
	v7.normal = Vec3f(1, -1, 0).normalized();
	v8.position = Vec3f(-offset, -offset, length_);
	v8.normal = Vec3f(-1, -1, 0).normalized();

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
		Vec3f color = distinct_colors[i % 6];
		v.color = color;
		i++;
	}

	return vertices;
}

std::vector<Vertex> JointMesh::getVertices()
{
	vector<Vertex> allVertices;
	
	int numRingVerts = 32;

	// first face to z = -length/2
	auto face1 = makeFace(numRingVerts, -depth_/2);
	vector<Vertex> face1Tris = face1.first;
	vector<Vertex> face1Ring = face1.second;
	allVertices.insert(allVertices.end(), face1Tris.begin(), face1Tris.end());

	// second face to z = depth/2
	auto face2 = makeFace(numRingVerts, depth_/2);
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
		v.color = Vec3f(1, 1, 0);
	}

	return allVertices;
}

// returns pair of triangle vertices, and ring vertices
std::pair<std::vector<Vertex>, std::vector<Vertex>> JointMesh::makeFace(int numRingVerts, float z)
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
