#include <iostream>
#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "Application.hpp"
#include "Utility.hpp"
#include "EventDispatcher.hpp"

using namespace Eigen;

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
}

Application::Application():
	drawmode_(DrawMode::MODE_MESH_CPU),
	shading_toggle_(false),
	shading_mode_changed_(false),
	camera_(Vector3f(0.0f, 0.0f, 1.5f))
{
	robot_ = std::make_unique<Robot>("src/resources/params.txt", Vector3f(1, 0, 0));

	// now robot knows how many joints it has, we can create sliders to control them
	joint_angle_controls_.resize(robot_->getNumJoints());
	prev_controls_.resize(robot_->getNumJoints());
}

void Application::createWindow(int width, int height) {
	// Init GLFW
	glfwInit();

	// Set all the required options for GLFW
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);

	// Create a GLFWwindow object that we can use for GLFW's functions
	window_ = glfwCreateWindow(width, height, "Robot Arm Simulator", nullptr, nullptr);
	EventDispatcher::SetApplication(this);

	if (nullptr == window_)
	{
		std::cerr << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		exit(EXIT_FAILURE);
	}

	glfwMakeContextCurrent(window_);

	// Set this to true so GLEW knows to use a modern approach to retrieving function pointers and extensions
	glewExperimental = GL_TRUE;
	// Initialize GLEW to setup the OpenGL Function pointers
	if (GLEW_OK != glewInit())
	{
		std::cerr << "Failed to initialize GLEW" << std::endl;
		glfwTerminate();
		exit(EXIT_FAILURE);
	}

	initRendering();
}

void Application::initRendering()
{
	// Create vertex attribute objects and buffers for vertex data.
	glGenVertexArrays(1, &gl_.simple_vao);
	glGenBuffers(1, &gl_.simple_vertex_buffer);

	// Point and line rendering setup
	glGenVertexArrays(1, &gl_.point_vao);
	glBindVertexArray(gl_.point_vao);
	glEnableVertexAttribArray(ATTRIB_POSITION);
	glVertexAttribPointer(ATTRIB_POSITION, 3, GL_FLOAT, GL_FALSE, sizeof(Eigen::Vector3f), (GLvoid*)0);
	glBindVertexArray(0);

	// Check for OpenGL errors.
	checkGlErrors();

	// Set up vertex attribute object for doing SSD on the CPU. The data will be loaded and re-loaded later, on each frame.
	glBindVertexArray(gl_.simple_vao);
	glBindBuffer(GL_ARRAY_BUFFER, gl_.simple_vertex_buffer);
	glEnableVertexAttribArray(ATTRIB_POSITION);
	glVertexAttribPointer(ATTRIB_POSITION, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)0);
	glEnableVertexAttribArray(ATTRIB_NORMAL);
	glVertexAttribPointer(ATTRIB_NORMAL, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)offsetof(Vertex, normal));
	glEnableVertexAttribArray(ATTRIB_COLOR);
	glVertexAttribPointer(ATTRIB_COLOR, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)offsetof(Vertex, color));
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	shader_ = std::make_unique<Shader>("simple_shader");
	shader_->use();

	checkGlErrors();

	// Get the IDs of the shader programs and their uniform input locations from OpenGL.
	gl_.simple_shader = shader_->ID;
	gl_.model = glGetUniformLocation(gl_.simple_shader, "model");
	gl_.view = glGetUniformLocation(gl_.simple_shader, "view");
	gl_.projection = glGetUniformLocation(gl_.simple_shader, "projection");
	gl_.lightPos = glGetUniformLocation(gl_.simple_shader, "lightPos");
	gl_.viewPos = glGetUniformLocation(gl_.simple_shader, "viewPos");
	gl_.lightColor = glGetUniformLocation(gl_.simple_shader, "lightColor");
	gl_.objectColor = glGetUniformLocation(gl_.simple_shader, "objectColor");

	checkGlErrors();
}

void Application::run(void) {
	assert(window_ != nullptr);

	glfwSetKeyCallback(window_, EventDispatcher::KeyboardCallback);
	glfwSetCursorPosCallback(window_, EventDispatcher::MouseMovedCallback);
	glfwSetMouseButtonCallback(window_, EventDispatcher::MouseButtonCallback);

	// Game loop
	while (!glfwWindowShouldClose(window_))
	{
		// time delta
		float dt = glfwGetTime();
		glfwSetTime(0);

		// Check if any events have been activiated (key pressed, mouse moved etc.) and call corresponding response functions
		glfwPollEvents();

		update(dt);
		render();
		
		// Swap the screen buffers
		glfwSwapBuffers(window_);
	}

	// Terminate GLFW, clearing any resources allocated by GLFW.
	glfwTerminate();
}

void Application::handleEvent(const Event& ev)
{
	camera_.handleEvent(ev);
}

void Application::update(float dt) {

	EventDispatcher::Update(window_, dt);
	robot_->update(dt);
}

void Application::render(void) {
	// Clear screen
	glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Enable depth testing
	glEnable(GL_DEPTH_TEST);

	// Adjust viewport and aspect ratio to window
	int screenWidth, screenHeight;
	glfwGetFramebufferSize(window_, &screenWidth, &screenHeight);
	glViewport(0, 0, screenWidth, screenHeight);
	float fAspect = float(screenWidth) / screenHeight;

	checkGlErrors();

	// Set up transform matrices.
	// They will be fed to the shader program via uniform variables.

	// World space -> clip space transform: simple projection and camera.
	Matrix4f C = camera_.getWorldToClip();

	Matrix4f P;
	static const float fNear = 0.3f, fFar = 4.0f;
	P.col(0) = Vector4f(1, 0, 0, 0);
	P.col(1) = Vector4f(0, fAspect, 0, 0);
	P.col(2) = Vector4f(0, 0, (fFar + fNear) / (fFar - fNear), 1);
	P.col(3) = Vector4f(0, 0, -2 * fFar * fNear / (fFar - fNear), 0);

	if (drawmode_ == DrawMode::MODE_SKELETON) {
		// Draw the skeleton as a set of joint positions, connecting lines,
		// and local coordinate systems at each joint.
		// Here we'll use old style (immediate mode) OpenGL as opposed to
		// modern GL where we always first load the data to a GPU buffer.
		// Immediate mode is convenient when we want to draw a small amount
		// of things without having to write much code, and performance is
		// not an issue: for instance, when you want to want to draw something
		// for debugging.

		// NOTE: all of this immediate mode shite doesn't work at all with modern opengl
		glUseProgram(0);

		checkGlErrors();

		glMatrixMode(GL_PROJECTION);

		checkGlErrors();

		glLoadMatrixf(P.data());

		

		glMatrixMode(GL_MODELVIEW);
		glLoadMatrixf(C.data());

		checkGlErrors();

		glEnable(GL_POINT_SMOOTH);
		glPointSize(15);

		checkGlErrors();

		// draw world origin frame:
		glLineWidth(2);
		float scale = 0.3;
		drawFrame(Eigen::Affine3f::Identity(), scale);

		checkGlErrors();

		// draw ray from camera
		glLineWidth(2);
		glBegin(GL_LINES);
		glColor3f(1, 0, 0); // red
		glVertex3f(ray_start_.x(), ray_start_.y(), ray_start_.z());
		glVertex3f(ray_end_.x(), ray_end_.y(), ray_end_.z());
		glEnd();
		glPointSize(20);
		glBegin(GL_POINTS);
		glColor3f(0, 0, 1); // blue
		glVertex3f(ray_end_.x(), ray_end_.y(), ray_end_.z());
		glEnd();

		checkGlErrors();

		robot_->renderSkeleton();
	}
	else // mesh mode
	{
		if (drawmode_ == DrawMode::MODE_MESH_WIREFRAME) {
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		}
		
		std::vector<Vertex> vertices = robot_->getMeshVertices();

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
			v.position -= Vector3f(grid_size * square_size / 2, 0, grid_size * square_size / 2);
		}
		vertices.insert(vertices.end(), planeVerts.begin(), planeVerts.end());

		checkGlErrors();

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

		Matrix4f model = Matrix4f::Identity();
		Vector3f viewpos = camera_.getPosition();
		temp += 0.01;

		glUniformMatrix4fv(gl_.model, 1, GL_FALSE, model.data());
		glUniformMatrix4fv(gl_.view, 1, GL_FALSE, C.data());
		glUniformMatrix4fv(gl_.projection, 1, GL_FALSE, P.data());

		glUniform3f(gl_.lightPos, 5 * sin(temp), 2, 5 * cos(temp));
		glUniform3f(gl_.viewPos, viewpos.x(), viewpos.y(), viewpos.z());
		glUniform3f(gl_.lightColor, 1, 1, 1);

		glBindVertexArray(gl_.simple_vao);
		glDrawArrays(GL_TRIANGLES, 0, (int)vertices.size());

		checkGlErrors();

		// draw ik target
		//glPointSize(20);
		//glBegin(GL_POINTS);
		//glColor3f(0, 0, 1); // blue
		//glVertex3f(ray_end_.x(), ray_end_.y(), ray_end_.z());
		//glEnd();

		if (drawmode_ == DrawMode::MODE_MESH_WIREFRAME) {
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		}

		glBindVertexArray(0);
		glUseProgram(0);
	}

	// Check for OpenGL errors.
	checkGlErrors();

	// Show status messages.
	//std::stringstream ss;
	//Vector3f tcp = rob_->getTcpWorldPosition();
	//Eigen::VectorXf speeds = rob_->getJointSpeeds();
	//Eigen::VectorXf tcpSpeed = rob_->getTcpSpeed();
	//ss << "TCP world position: (" << tcp.x() << ", " << tcp.y() << ", " << tcp.z() << ")" << endl;
	//ss << "Joint speeds: " << speeds(0) << ", " << speeds(1) << endl;
	//ss << "TCP speed: " << tcpSpeed(0) << ", " << tcpSpeed(1) << ", " << tcpSpeed(2) << " Angular: " << tcpSpeed(3) << ", " << tcpSpeed(4) << ", " << tcpSpeed(5) << endl;
	//ss << endl;

	//common_ctrl_.message(ss.str().c_str(), "info");
}


