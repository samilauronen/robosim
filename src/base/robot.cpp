#include "robot.hpp"
#include "utility.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

using namespace std;

// TODO:
/*
* Don't create meshes again for every frame, that just sucks!
* Split the mesh code and other drawing code to another class, maybe RobotGraphics
* Make it possible to have offset and link length in same parameter and have it look okay on the skeleton and mesh views
* Make prismatic joints possible?
* Inverse kinematics, given a point (first programmatically, later by mouse click in the world)
* Maybe have a toggle for drawing a wireframe-ball around the robot, visualizing it's range
* Jacobians, especially inverse, for trajectories!
* Dynamics???
*/

Robot::Robot(std::string dh_param_filename, FW::Vec3f location)
	: selected_joint_(0),
	filename_(dh_param_filename)
{
	// base to 0 frame
	Mat3f rotation = Mat3f::rotation(Vec3f(1, 0, 0), -FW_PI / 2);
	baseToZero_ = combineToMat4f(rotation, Vec3f(0, 0, 0));

	loadDhParams();
	buildModel();

	// set the transformation from world to base, no rotation here, just set the location
	worldToBase_ = combineToMat4f(Mat3f::rotation(Vec3f(0,0,0),0), location);

}

void Robot::renderSkeleton()
{
	glEnable(GL_POINT_SMOOTH);
	glPointSize(15);
	
	updateToWorldTransforms();

	// draw world origin frame:
	glLineWidth(2);
	glBegin(GL_LINES);
	float scale = 0.3;
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
	glEnd();

	// draw link frames
	for (int i = 0; i < links_.size(); i++) {
		Mat4f current_trans = links_[i].to_world;

		bool is_tcp_frame = i + 1 == links_.size();

		// world-position of the current frame origin
		Vec3f pos = current_trans * Vec3f(0, 0, 0);

		// draw a point at the origin of the new frame
		// only draw if the current frame location is also a joint
		if (!is_tcp_frame) {
			glBegin(GL_POINTS);
			if (selected_joint_ == i)
				glColor3f(1.0f, 0.2f, 0.2f);
			else
				glColor3f(1.0f, 1.0f, 1.0f);
			glVertex3f(pos.x, pos.y, pos.z);
			glEnd();
		}
		

		// NOTE: disable this if it causes problems with any calculations in the future!
		// rotate frames with joint rotation, even though that doesnt follow DH standard?
		bool showJointRotation = false;
		Mat3f jointRotation;
		if (showJointRotation && i < links_.size()) {
			jointRotation = Mat3f::rotation(Vec3f(0, 0, 1), links_[i].rotation);
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
		glBegin(GL_LINES);
		Mat3f frame_rotation= current_trans.getXYZ() * jointRotation;
		Vec3f new_i = frame_rotation.getCol(0).normalized() * scale;
		Vec3f new_j = frame_rotation.getCol(1).normalized() * scale;
		Vec3f new_k = frame_rotation.getCol(2).normalized() * scale;
		glColor3f(1, 0, 0); // red
		glVertex3f(pos.x, pos.y, pos.z);
		glVertex3f(pos.x + new_i.x, pos.y + new_i.y, pos.z + new_i.z);
		glColor3f(0, 1, 0); // green
		glVertex3f(pos.x, pos.y, pos.z);
		glVertex3f(pos.x + new_j.x, pos.y + new_j.y, pos.z + new_j.z);
		glColor3f(0, 0, 1); // blue
		glVertex3f(pos.x, pos.y, pos.z);
		glVertex3f(pos.x + new_k.x, pos.y + new_k.y, pos.z + new_k.z);
		glEnd();

		glLineWidth(1);
		glBegin(GL_LINES);
		// if parent frame exists, draw a line from it to the current frame (symbolizes a link)
		int parent = i - 1;
		if (parent != -1) {
			glColor3f(1, 1, 1);
			
			
			Vec4f t = links_[i].link_matrix.getCol(3);
			Vec3f parent_world_pos = links_[parent].to_world * Vec3f(0, 0, 0);

			// TODO: figure out how to make the "90 degree link" style when we have offset and link length simultaneously
			//glVertex3f(pos.x, pos.y, pos.z);
			//glVertex3f(pos.x, pos.y, pos.z - t.z);

			//glVertex3f(pos.x, pos.y, pos.z - t.z);
			//glVertex3f(pos.x - t.x, pos.y, pos.z - t.z);

			glVertex3f(pos.x, pos.y, pos.z);
			glVertex3f(parent_world_pos.x, parent_world_pos.y, parent_world_pos.z);
		}

		glEnd();
	}
}

vector<Vertex> Robot::getMeshVertices() 
{
	vector<Mat4f> transforms = getToWorldTransforms();
	vector<Vertex> allVertices;

	bool isOffset = false;
	float linkLength = 0;
	float init_link_thickness = 0.1f;
	float init_joint_radius = 0.07f;
	float init_joint_length = 0.2f;
	float size_reduction_factor = 0.8;

	// link meshes
	for (unsigned i = 0; i < transforms.size(); i++) {
		isOffset = !(FW::abs(links_[i].p.a) > 0.00001);  // links use either offset (d) or link length (a). Both are not allowed by this implementation

		// create link meshes
		linkLength = FW::abs(isOffset ? links_[i].p.d : links_[i].p.a);
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

		// set joint mesh color different if it is selected
		if (selected_joint_ == i) {
			for (auto& v : meshVertices) {
				v.color = Vec3f(1.0f, 0.2f, 0.2f);
			}
		}

		// orient along the joint's z-axis 
		Mat3f jointBasisVecs = transforms[i].getXYZ();
		Mat3f orientation = jointBasisVecs;

		// translate to world position of joint 
		Vec3f translation = transforms[i] * Vec3f(0, 0, 0);

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
	// first update the link matrix, since rotation may have changed
	for (int i = 0; i < links_.size(); i++) {
		Link& j = links_[i];

		// TODO: revert this back to multiplication of screws
		Mat4f prev_to_this;
		Vec4f col0 = Vec4f(FW::cos(j.rotation), FW::sin(j.rotation), 0, 0);
		Vec4f col1 = Vec4f(-FW::sin(j.rotation) * FW::cos(j.p.alpha), FW::cos(j.rotation) * FW::cos(j.p.alpha), FW::sin(j.p.alpha), 0);
		Vec4f col2 = Vec4f(FW::sin(j.rotation) * FW::sin(j.p.alpha), -FW::cos(j.rotation) * FW::sin(j.p.alpha), FW::cos(j.p.alpha), 0);
		Vec4f col3 = Vec4f(j.p.a * FW::cos(j.rotation), j.p.a * FW::sin(j.rotation), j.p.d, 1);
		prev_to_this.setCol(0, col0);
		prev_to_this.setCol(1, col1);
		prev_to_this.setCol(2, col2);
		prev_to_this.setCol(3, col3);

		j.link_matrix = prev_to_this;
	}

	Mat4f previous_transform;
	Mat4f current_transform;

	// now chain the link transformations like so:
	// for link 0 the result should be W_T_0 (which is W_T_B * B_T_0)
	// for link 1: W_T_1
	// for link 2: W_T_2
	// etc...
	// index always tells which frame is the result of the transformation

	previous_transform = worldToBase_ * baseToZero_;
	for (int i = 0; i < links_.size(); i++) {
		current_transform = previous_transform * links_[i].link_matrix;
		links_[i].to_world = current_transform;
		previous_transform = current_transform;
	}
}

const std::vector<Link>& Robot::getLinks()
{
	updateToWorldTransforms();
	return links_;
}

std::vector<FW::Mat4f> Robot::getToWorldTransforms()
{
	updateToWorldTransforms();
	vector<Mat4f> transforms;
	for (int i = 0; i < links_.size(); i++) {
		transforms.push_back(links_[i].to_world);
	}
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

	int i = 0;

	Link link_zero;
	link_zero.p = DhParam{};
	link_zero.rotation = 0;
	link_zero.is_locked = true;	// should not be rotated
	links_.push_back(link_zero);

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

		Link l;
		l.p = p;
		l.rotation = 0;
		links_.push_back(l);
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
