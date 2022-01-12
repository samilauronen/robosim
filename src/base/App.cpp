#define _CRT_SECURE_NO_WARNINGS

#include "App.hpp"
#include "base/Main.hpp"
#include "gpu/GLContext.hpp"
#include "gpu/Buffer.hpp"
#include "Utility.hpp"

#include <array>
#include <cassert>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstddef>
#include <type_traits>
#include <numeric>
#include <map>
#include <iomanip>

using namespace FW;
using namespace Eigen;
using namespace std;

namespace {

enum VertexShaderAttributeLocations
{
	ATTRIB_POSITION = 0,
	ATTRIB_NORMAL = 1,
	ATTRIB_COLOR = 2,
	ATTRIB_JOINTS1 = 3,
	ATTRIB_JOINTS2 = 4,
	ATTRIB_WEIGHTS1 = 5,
	ATTRIB_WEIGHTS2 = 6
};

} // namespace

App::App(void)
:   common_ctrl_			(CommonControls::Feature_Default & ~CommonControls::Feature_RepaintOnF5),
	camera_ctrl_			(&common_ctrl_, CameraControls::Feature_None),
	drawmode_				(MODE_SKELETON),
	shading_toggle_			(false),
	shading_mode_changed_	(false)
{
	rob_ = std::make_unique<Robot>("src/base/params.txt", Vector3f(1, 0, 0));

	// now robot knows how many joints it has, we can create sliders to control them
	joint_angle_controls_.resize(rob_->getNumJoints());
	prev_controls_.resize(rob_->getNumJoints());

	initRendering();
		
	common_ctrl_.showFPS(true);
	common_ctrl_.addToggle((S32*)&drawmode_, MODE_SKELETON,			FW_KEY_1, "Draw joints and bones (1)");
	common_ctrl_.addToggle((S32*)&drawmode_, MODE_MESH_CPU,			FW_KEY_2, "Draw mesh, SSD on CPU (2)");
	common_ctrl_.addToggle((S32*)&drawmode_, MODE_MESH_WIREFRAME,	FW_KEY_3, "EXTRA: Draw mesh, SSD on GPU (3)");
	common_ctrl_.addSeparator();
	common_ctrl_.addToggle(&shading_toggle_,						FW_KEY_T, "Toggle shading mode (T)",	&shading_mode_changed_);

	// NOTE: it is possible to modify slider values from other sources than the slider, but that requires some addtions to FW
	// common controls has setSliderValue but that requires Slider*, maybe make addSlider return that?
	// would be cool to have the sliders react while the robot is tracing some path or doing inverse kinematics move animation
	for (int i = 0; i < rob_->getNumJoints(); i++) {
		common_ctrl_.beginSliderStack();
		common_ctrl_.addSlider(joint_angle_controls_.data() + i, -FW_PI, FW_PI, false, FW_KEY_NONE, FW_KEY_NONE, "%.2f");
		common_ctrl_.endSliderStack();
	}

	window_.setTitle("Robot Simulator!");

	window_.addListener(this);
	window_.addListener(&common_ctrl_);

	camera_ctrl_.setSpeed(1.75f);

	window_.setSize( Vec2i(800, 800) );

	time_end_ = currentTimeMicros();
}

bool App::handleEvent(const Window::Event& ev) {	

	uint64_t time_start = currentTimeMicros();

	uint64_t dt_micros = time_start - time_end_;
	float dt_millis = (float)dt_micros / 1000;

	rob_->update(dt_millis);

	time_end_ = currentTimeMicros();

	if (shading_mode_changed_) {
		common_ctrl_.message(shading_toggle_ ?
			"Directional light shading using normals; direction to light (0.5, 0.5, -0.6)" :
			"Joint weight coloring");
		shading_mode_changed_ = false;
	}

	if (ev.type == Window::EventType_KeyDown) {
		if (ev.key == FW_KEY_MOUSE_MIDDLE) {
			// convert to normalized image coordinates
			float x = ev.mousePos.x / (float)window_.getSize().x;
			float y = ev.mousePos.y / (float)window_.getSize().y;
			x = x * 2 - 1;
			y = -(y * 2 - 1);

			// grab camera attributes
			float fov = camera_ctrl_.getFOV() * FW_PI / 180; // damn thing was in degrees!
			Vec3f up = camera_ctrl_.getUp();
			Vec3f direction = camera_ctrl_.getForward();
			Vec3f horizontal = direction.cross(up);
			Vec3f position = camera_ctrl_.getPosition();

			// calculate distance from "image plane"
			float dist = 1 / FW::tan(fov / 2);

			// create ray
			FW::Vec3f x_vec = x * horizontal.normalized();
			FW::Vec3f y_vec = y * up.normalized();
			FW::Vec3f dist_vec = dist * direction;
			FW::Vec3f ray_vec = (dist_vec + x_vec + y_vec).normalized();

			// ray should ge going from [position] towards [ray_vec]
			float t = 1;
			ray_start_ = position;
			ray_end_ = position + ray_vec * t;

			rob_->setTargetTcpPosition(Vector3f(ray_end_.x, ray_end_.y, ray_end_.z));
		}
	}

	if (ev.type == Window::EventType_KeyDown) {
		if (ev.key == FW_KEY_ENTER) {
			ik_ = true;
		}
	}

	if (ev.type == Window::EventType_Close)	{
		window_.showModalMessage("Exiting...");
		delete this;
		return true;
	}

	// set robot joint rotations to slider values
	for (int i = 0; i < joint_angle_controls_.size(); i++) {
		float curr = joint_angle_controls_[i];
		float prev = prev_controls_[i];
		if (abs(curr - prev) > 0.0001) {
			rob_->setJointTargetAngle(i, joint_angle_controls_[i]);
			prev_controls_[i] = curr;
		}
	}

	camera_ctrl_.handleEvent(ev);

	window_.setVisible(true);
	if (ev.type == Window::EventType_Paint)
		render();
	window_.repaint();
	return false;
}

void App::initRendering()
{
	// Ask the Nvidia framework for the GLContext object associated with the window.
	// As a side effect, this initializes the OpenGL context and lets us call GL functions.
	auto ctx = window_.getGL();
	
	// Create vertex attribute objects and buffers for vertex data.
	glGenVertexArrays(1, &gl_.simple_vao);
	glGenVertexArrays(1, &gl_.ssd_vao);
	glGenBuffers(1, &gl_.simple_vertex_buffer);
	glGenBuffers(1, &gl_.ssd_vertex_buffer);

	// Point and line rendering setup
	glGenVertexArrays(1, &gl_.point_vao);
	glBindVertexArray(gl_.point_vao);
	glEnableVertexAttribArray(ATTRIB_POSITION);
	glVertexAttribPointer(ATTRIB_POSITION, 3, GL_FLOAT, GL_FALSE, sizeof(Eigen::Vector3f), (GLvoid*)0);
	glBindVertexArray(0);
	
	// Check for OpenGL errors.
	GLContext::checkErrors();

	// Set up vertex attribute object for doing SSD on the CPU. The data will be loaded and re-loaded later, on each frame.
	glBindVertexArray(gl_.simple_vao);
	glBindBuffer(GL_ARRAY_BUFFER, gl_.simple_vertex_buffer);
	glEnableVertexAttribArray(ATTRIB_POSITION);
	glVertexAttribPointer(ATTRIB_POSITION, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*) 0);
	glEnableVertexAttribArray(ATTRIB_NORMAL);
	glVertexAttribPointer(ATTRIB_NORMAL, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*) offsetof(Vertex, normal));
	glEnableVertexAttribArray(ATTRIB_COLOR);
	glVertexAttribPointer(ATTRIB_COLOR, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*) offsetof(Vertex, color));
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	// Compile and link the shader programs.

	auto simple_prog = new GLContext::Program(
		"#version 330\n"
		FW_GL_SHADER_SOURCE(
		layout(location = 0) in vec4 aPosition;
		layout(location = 1) in vec3 aNormal;
		layout(location = 2) in vec3 aColor;

		out vec3 FragPos;
		out vec3 Normal;
		out vec3 vColor;

		uniform mat4 model;
		uniform mat4 view;
		uniform mat4 projection;

		void main()
		{
			FragPos = vec3(model * aPosition);
			Normal = mat3(transpose(inverse(model))) * aNormal;
			vColor = aColor;

			gl_Position = projection * view * vec4(FragPos, 1.0);
		}
		),
		"#version 330\n"
		FW_GL_SHADER_SOURCE(

		out vec4 FragColor;

		in vec3 Normal;
		in vec3 FragPos;
		in vec3 vColor;

		uniform vec3 lightPos;
		uniform vec3 viewPos;
		uniform vec3 lightColor;

		void main()
		{
			// ambient
			float ambientStrength = 0.2;
			vec3 ambient = ambientStrength * lightColor;

			// diffuse
			vec3 norm = normalize(Normal);
			vec3 lightDir = normalize(lightPos - FragPos);
			float diff = max(dot(norm, lightDir), 0.0);
			vec3 diffuse = diff * lightColor;

			// specular
			float specularStrength = 0.3;
			vec3 viewDir = normalize(viewPos - FragPos);
			vec3 reflectDir = reflect(-lightDir, norm);
			float spec = pow(max(dot(viewDir, reflectDir), 0.0), 4);
			vec3 specular = specularStrength * spec * lightColor;

			vec3 result = (ambient + diffuse + specular) * vColor;
			FragColor = vec4(result, 1.0);
		}
		));
	ctx->setProgram("simple_shader", simple_prog);

	// Get the IDs of the shader programs and their uniform input locations from OpenGL.
	gl_.simple_shader = simple_prog->getHandle();
	gl_.model = glGetUniformLocation(gl_.simple_shader, "model");
	gl_.view  = glGetUniformLocation(gl_.simple_shader, "view");
	gl_.projection = glGetUniformLocation(gl_.simple_shader, "projection");
	gl_.lightPos = glGetUniformLocation(gl_.simple_shader, "lightPos");
	gl_.viewPos = glGetUniformLocation(gl_.simple_shader, "viewPos");
	gl_.lightColor = glGetUniformLocation(gl_.simple_shader, "lightColor");
	gl_.objectColor = glGetUniformLocation(gl_.simple_shader, "objectColor");
}

void App::render() {
	// Clear screen
	glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Enable depth testing
	glEnable(GL_DEPTH_TEST);

	// Adjust viewport and aspect ratio to window
	auto sz = window_.getSize();
	float fAspect = float(sz.x)/sz.y;
	glViewport(0, 0, sz.x, sz.y);
	
	// Set up transform matrices.
	// They will be fed to the shader program via uniform variables.

	// World space -> clip space transform: simple projection and camera.
	Mat4f C = camera_ctrl_.getWorldToClip();

	Mat4f P;
	static const float fNear = 0.3f, fFar = 4.0f;
	P.setCol(0, Vec4f(1, 0, 0, 0));
	P.setCol(1, Vec4f(0, fAspect, 0, 0));
	P.setCol(2, Vec4f(0, 0, (fFar+fNear)/(fFar-fNear), 1));
	P.setCol(3, Vec4f(0, 0, -2*fFar*fNear/(fFar-fNear), 0));
	Mat4f world_to_clip = P * C;

	Eigen::Matrix4f EP = toEigenMatrix(P);

	if (drawmode_ == MODE_SKELETON) {
		// Draw the skeleton as a set of joint positions, connecting lines,
		// and local coordinate systems at each joint.
		// Here we'll use old style (immediate mode) OpenGL as opposed to
		// modern GL where we always first load the data to a GPU buffer.
		// Immediate mode is convenient when we want to draw a small amount
		// of things without having to write much code, and performance is
		// not an issue: for instance, when you want to want to draw something
		// for debugging.
		glUseProgram(0);
		glMatrixMode(GL_PROJECTION);
		glLoadMatrixf(EP.data());
		glMatrixMode(GL_MODELVIEW);
		glLoadMatrixf(&C(0,0));

		glEnable(GL_POINT_SMOOTH);
		glPointSize(15);

		// draw world origin frame:
		glLineWidth(2);
		float scale = 0.3;
		drawFrame(Eigen::Affine3f::Identity(), scale);

		// draw ray from camera
		glLineWidth(2);
		glBegin(GL_LINES);
		glColor3f(1, 0, 0); // red
		glVertex3f(ray_start_.x, ray_start_.y, ray_start_.z);
		glVertex3f(ray_end_.x, ray_end_.y, ray_end_.z);
		glEnd();
		glPointSize(20);
		glBegin(GL_POINTS);
		glColor3f(0, 0, 1); // blue
		glVertex3f(ray_end_.x, ray_end_.y, ray_end_.z);
		glEnd();

		rob_->renderSkeleton();
	}
	else // mesh mode
	{
		if (drawmode_ == MODE_MESH_WIREFRAME) {
			glPolygonMode(GL_FRONT, GL_LINE);
			glPolygonMode(GL_BACK, GL_LINE);
		}
		std::vector<Vertex> vertices = rob_->getMeshVertices();

		float square_size = 0.2;
		float grid_size = 50;

		std::vector<Vertex> planeVerts;

		for (int i = 0; i < grid_size; i++) {
			for (int j = 0; j < grid_size; j++) {
				Vector3f color = (i + j) % 2 == 0 ? Vector3f(0.8, 0.8, 0.8) : Vector3f(0.3, 0.3, 0.3);

				// left triangle
				Vertex p1, p2, p3;
				p1.position = Vector3f(i * square_size, 0, j * square_size);
				p2.position = Vector3f(i * square_size, 0, (j + 1) * square_size);
				p3.position = Vector3f((i + 1) * square_size, 0, j * square_size);

				p1.normal = p2.normal = p3.normal = Vector3f(0, 1, 0);
				p1.color = p2.color = p3.color = color;
				planeVerts.insert(planeVerts.end(), { p1,p2,p3 });

				// right triangle
				p1.position = Vector3f(i * square_size, 0, (j + 1) * square_size);
				p2.position = Vector3f((i + 1) * square_size, 0, (j + 1) * square_size);
				p3.position = Vector3f((i + 1) * square_size, 0, j * square_size);

				p1.normal = p2.normal = p3.normal = Vector3f(0, 1, 0);
				p1.color = p2.color = p3.color = color;
				planeVerts.insert(planeVerts.end(), { p1,p2,p3 });
			}
		}
		for (Vertex& v : planeVerts) {
			v.position -= Vector3f(grid_size*square_size / 2, 0, grid_size*square_size / 2);
		}
		vertices.insert(vertices.end(), planeVerts.begin(), planeVerts.end());

		glBindBuffer(GL_ARRAY_BUFFER, gl_.simple_vertex_buffer);
		glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * vertices.size(), vertices.data(), GL_STATIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glUseProgram(gl_.simple_shader);

		gl_.model = glGetUniformLocation(gl_.simple_shader, "model");
		gl_.view = glGetUniformLocation(gl_.simple_shader, "view");
		gl_.projection = glGetUniformLocation(gl_.simple_shader, "projection");
		gl_.lightPos = glGetUniformLocation(gl_.simple_shader, "lightPos");
		gl_.viewPos = glGetUniformLocation(gl_.simple_shader, "viewPos");
		gl_.lightColor = glGetUniformLocation(gl_.simple_shader, "lightColor");
		gl_.objectColor = glGetUniformLocation(gl_.simple_shader, "objectColor");

		Mat4f model;
		Vec3f viewpos = camera_ctrl_.getPosition();
		temp += 0.01;

		glUniformMatrix4fv(gl_.model, 1, GL_FALSE, model.getPtr());
		glUniformMatrix4fv(gl_.view, 1, GL_FALSE, C.getPtr());
		glUniformMatrix4fv(gl_.projection, 1, GL_FALSE, P.getPtr());

		glUniform3f(gl_.lightPos, 5*sin(temp), 2, 5 * cos(temp));
		glUniform3f(gl_.viewPos,  viewpos.x, viewpos.y, viewpos.z);
		glUniform3f(gl_.lightColor, 1, 1, 1);

		glBindVertexArray(gl_.simple_vao);
		glDrawArrays(GL_TRIANGLES, 0, (int)vertices.size());

		// draw ik target
		glPointSize(20);
		glBegin(GL_POINTS);
		glColor3f(0, 0, 1); // blue
		glVertex3f(ray_end_.x, ray_end_.y, ray_end_.z);
		glEnd();

		// reset polygon mode
		glPolygonMode(GL_FRONT, GL_FILL);
		glPolygonMode(GL_BACK, GL_FILL);

		glBindVertexArray(0);
		glUseProgram(0);
	}
	
	// Check for OpenGL errors.
	GLContext::checkErrors();

	GLenum err;
	while ((err = glGetError()) != GL_NO_ERROR)
	{
		cerr << "error: " << err << endl;
	}
	
	// Show status messages.
	std::stringstream ss;
	Vector3f tcp = rob_->getTcpWorldPosition();
	Eigen::VectorXf speeds = rob_->getJointSpeeds();
	Eigen::VectorXf tcpSpeed = rob_->getTcpSpeed();
	ss << "TCP world position: ("  << tcp.x() << ", " << tcp.y() << ", " << tcp.z() << ")" << endl;
	ss << "Joint speeds: " << speeds(0)  << ", " << speeds(1) << endl;
	ss << "TCP speed: " << tcpSpeed(0) << ", " << tcpSpeed(1) << ", " << tcpSpeed(2) << " Angular: " << tcpSpeed(3) << ", " << tcpSpeed(4) << ", " << tcpSpeed(5) << endl;
	ss << endl;

	common_ctrl_.message(ss.str().c_str(), "info");
}

void FW::init(void) {
	new App;
}
