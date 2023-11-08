#include <iostream>
#include <sstream>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#define GLEW_STATIC
#include "GL/glew.h"
#include "GLFW/glfw3.h"

#include "Application.hpp"
#include "Utility.hpp"
#include "meshes/BoxMesh.hpp"
#include "meshes/SphereMesh.hpp"

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
	drawmode_(DrawMode::MODE_MESH),
	shading_toggle_(false),
	shading_mode_changed_(false),
	camera_(Vector3f(1, 0, 0), 4.0f, M_PI / 4, M_PI / 6)
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
	glfwWindowHint(GLFW_MAXIMIZED, GL_TRUE);


	// Create a GLFWwindow object that we can use for GLFW's functions
	window_ = glfwCreateWindow(width, height, "Robot Arm Simulator", nullptr, nullptr);
	if (nullptr == window_)
	{
		std::cerr << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		exit(EXIT_FAILURE);
	}
	glfwMakeContextCurrent(window_);

	glfwSetWindowUserPointer(window_, this);

	// Setup event handling
	EventDispatcher::SetApplication(this);
	glfwSetKeyCallback(window_, EventDispatcher::KeyboardCallback);
	glfwSetCursorPosCallback(window_, EventDispatcher::MouseMovedCallback);
	glfwSetMouseButtonCallback(window_, EventDispatcher::MouseButtonCallback);
	glfwSetScrollCallback(window_, EventDispatcher::MouseScrolledCallback);

	// Imgui setup
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();

	{
		ImGuiStyle* style = &ImGui::GetStyle();

		style->WindowPadding = ImVec2(15, 15);
		style->WindowRounding = 5.0f;
		style->FramePadding = ImVec2(5, 5);
		style->FrameRounding = 4.0f;
		style->ItemSpacing = ImVec2(12, 8);
		style->ItemInnerSpacing = ImVec2(8, 6);
		style->IndentSpacing = 25.0f;
		style->ScrollbarSize = 15.0f;
		style->ScrollbarRounding = 9.0f;
		style->GrabMinSize = 5.0f;
		style->GrabRounding = 3.0f;

		style->Colors[ImGuiCol_Text] = ImVec4(0.80f, 0.80f, 0.83f, 1.00f);
		style->Colors[ImGuiCol_TextDisabled] = ImVec4(0.24f, 0.23f, 0.29f, 1.00f);
		style->Colors[ImGuiCol_WindowBg] = ImVec4(0.06f, 0.05f, 0.07f, 1.00f);
		style->Colors[ImGuiCol_PopupBg] = ImVec4(0.07f, 0.07f, 0.09f, 1.00f);
		style->Colors[ImGuiCol_Border] = ImVec4(0.80f, 0.80f, 0.83f, 0.88f);
		style->Colors[ImGuiCol_BorderShadow] = ImVec4(0.92f, 0.91f, 0.88f, 0.00f);
		style->Colors[ImGuiCol_FrameBg] = ImVec4(0.10f, 0.09f, 0.12f, 1.00f);
		style->Colors[ImGuiCol_FrameBgHovered] = ImVec4(0.24f, 0.23f, 0.29f, 1.00f);
		style->Colors[ImGuiCol_FrameBgActive] = ImVec4(0.56f, 0.56f, 0.58f, 1.00f);
		style->Colors[ImGuiCol_TitleBg] = ImVec4(0.10f, 0.09f, 0.12f, 1.00f);
		style->Colors[ImGuiCol_TitleBgCollapsed] = ImVec4(1.00f, 0.98f, 0.95f, 0.75f);
		style->Colors[ImGuiCol_TitleBgActive] = ImVec4(0.07f, 0.07f, 0.09f, 1.00f);
		style->Colors[ImGuiCol_MenuBarBg] = ImVec4(0.10f, 0.09f, 0.12f, 1.00f);
		style->Colors[ImGuiCol_ScrollbarBg] = ImVec4(0.10f, 0.09f, 0.12f, 1.00f);
		style->Colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.80f, 0.80f, 0.83f, 0.31f);
		style->Colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.56f, 0.56f, 0.58f, 1.00f);
		style->Colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.06f, 0.05f, 0.07f, 1.00f);
		style->Colors[ImGuiCol_CheckMark] = ImVec4(0.80f, 0.80f, 0.83f, 0.31f);
		style->Colors[ImGuiCol_SliderGrab] = ImVec4(0.80f, 0.80f, 0.83f, 0.31f);
		style->Colors[ImGuiCol_SliderGrabActive] = ImVec4(0.06f, 0.05f, 0.07f, 1.00f);
		style->Colors[ImGuiCol_Button] = ImVec4(0.10f, 0.09f, 0.12f, 1.00f);
		style->Colors[ImGuiCol_ButtonHovered] = ImVec4(0.24f, 0.23f, 0.29f, 1.00f);
		style->Colors[ImGuiCol_ButtonActive] = ImVec4(0.56f, 0.56f, 0.58f, 1.00f);
		style->Colors[ImGuiCol_Header] = ImVec4(0.10f, 0.09f, 0.12f, 1.00f);
		style->Colors[ImGuiCol_HeaderHovered] = ImVec4(0.56f, 0.56f, 0.58f, 1.00f);
		style->Colors[ImGuiCol_HeaderActive] = ImVec4(0.06f, 0.05f, 0.07f, 1.00f);
		style->Colors[ImGuiCol_ResizeGrip] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
		style->Colors[ImGuiCol_ResizeGripHovered] = ImVec4(0.56f, 0.56f, 0.58f, 1.00f);
		style->Colors[ImGuiCol_ResizeGripActive] = ImVec4(0.06f, 0.05f, 0.07f, 1.00f);
		style->Colors[ImGuiCol_PlotLines] = ImVec4(0.40f, 0.39f, 0.38f, 0.63f);
		style->Colors[ImGuiCol_PlotLinesHovered] = ImVec4(0.25f, 1.00f, 0.00f, 1.00f);
		style->Colors[ImGuiCol_PlotHistogram] = ImVec4(0.40f, 0.39f, 0.38f, 0.63f);
		style->Colors[ImGuiCol_PlotHistogramHovered] = ImVec4(0.25f, 1.00f, 0.00f, 1.00f);
		style->Colors[ImGuiCol_TextSelectedBg] = ImVec4(0.25f, 1.00f, 0.00f, 0.43f);
	}

	ImGui_ImplGlfw_InitForOpenGL(window_, true);
	ImGui_ImplOpenGL3_Init("#version 330");

	// Set this to true so GLEW knows to use a modern approach to retrieving function pointers and extensions
	glewExperimental = GL_TRUE;
	// Initialize GLEW to setup the OpenGL Function pointers
	if (GLEW_OK != glewInit())
	{
		std::cerr << "Failed to initialize GLEW" << std::endl;
		glfwTerminate();
		exit(EXIT_FAILURE);
	}

	// registe callback (old)
	// glfwSetMouseButtonCallback(window_,
	// 	[](GLFWwindow* w, int button, int action, int mods)
	// 	{
	// 		static_cast<Application*>(glfwGetWindowUserPointer(w))->mouseButtonCallback(w, button, action, mods);
	// 	}
	// );

	initRendering();
}

void Application::handleEvent(const Event& ev)
{
	camera_.handleEvent(ev);
	
	// handle right mouse clicks
	if (ev.type == EventType::KEY_DOWN && ev.key == GLFW_MOUSE_BUTTON_RIGHT)
	{
		setIkTarget();
		running_ik_solution_ = true;
	}
}

void Application::initRendering()
{
	// Create vertex attribute objects and buffers for vertex data.
	glGenVertexArrays(1, &gl_.simple_vao);
	glGenBuffers(1, &gl_.simple_vertex_buffer);

	// Check for OpenGL errors.
	checkGlErrors();

	// Set up vertex attribute object
	glBindVertexArray(gl_.simple_vao);
	glBindBuffer(GL_ARRAY_BUFFER, gl_.simple_vertex_buffer);
	glEnableVertexAttribArray(ATTRIB_POSITION);
	glVertexAttribPointer(ATTRIB_POSITION, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)0);
	glEnableVertexAttribArray(ATTRIB_NORMAL);
	glVertexAttribPointer(ATTRIB_NORMAL, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)offsetof(Vertex, normal));
	glEnableVertexAttribArray(ATTRIB_COLOR);
	glVertexAttribPointer(ATTRIB_COLOR, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)offsetof(Vertex, color));
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

	bool show_demo_window = true;
	bool show_another_window = false;
	ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

	std::vector<DhParam> robot_params = robot_->getDhParams();

	float last_time = glfwGetTime();
	float curr_time;

	// program loop
	while (!glfwWindowShouldClose(window_))
	{
		glfwPollEvents();

		// calculate time delta
		curr_time = glfwGetTime();
		float dt = curr_time - last_time;
		last_time = curr_time;

		// ============= ImGui  ================
		{
			using namespace ImGui;
			ImGui_ImplOpenGL3_NewFrame();
			ImGui_ImplGlfw_NewFrame();
			NewFrame();

			Begin("Joint angle controls:");
			for (int i = 0; i < robot_->getNumJoints(); i++) {
				PushID(i);
				Text("joint %d", i);
				SameLine();
				SliderAngle("", &joint_angle_controls_[i]);
				// user manually grabbing a slider aborts the running IK solution
				if (ImGui::IsItemActive() && running_ik_solution_)
				{
					abortRunningIkSolution();
				}
				PopID();
			}
			End();

			Begin("Joint PID controller gains:");
			SliderFloat("P", &pid_p, 0, 0.1);
			SliderFloat("I", &pid_i, 0, 0.05);
			SliderFloat("D", &pid_d, 0, 0.05);
			robot_->setJointControllerPidGains(pid_p, pid_i, pid_d);
			End();

			Begin("TCP info");
			Vector3f tcp_pos = robot_->getTcpWorldPosition();
			Vector3f tcp_target = robot_->getTargetTcpPosition();
			VectorXf tcp_speed = robot_->getTcpSpeed();
			Columns(2);
			Text("TCP position:");
			NextColumn();
			Text("x: % 2.2f, y: % 2.2f, z: % 2.2f", tcp_pos.x(), tcp_pos.y(), tcp_pos.z());
			Separator();
			NextColumn();
			Text("TCP target:");
			NextColumn();
			Text("x: % 2.2f, y: % 2.2f, z: % 2.2f", tcp_target.x(), tcp_target.y(), tcp_target.z());
			Separator();
			NextColumn();
			Text("TCP speed:");
			NextColumn();
			Text("x: % 2.2f, y: % 2.2f, z: % 2.2f\nrx: % 2.2f, ry: % 2.2f, rz: % 2.2f",
				tcp_speed[0], tcp_speed[1], tcp_speed[2], tcp_speed[3], tcp_speed[4], tcp_speed[5]);
			End();

			Begin("Parameter editor");
			if (BeginTable("paramtable", robot_->getNumJoints() + 1)) {
				// header row
				TableHeadersRow();
				TableNextColumn();
				Text("");
				TableNextColumn();
				Text("a");
				TableNextColumn();
				Text("d");
				TableNextColumn();
				Text("alpha");

				// table content rows
				for (int i = 0; i < robot_->getNumJoints(); i++) {
					PushID(i);
					TableNextRow();
					TableNextColumn();
					//TableSetupColumn("", 0, 10);
					Text("joint %d", i);
					TableNextColumn();
					//TableSetupColumn("a", 0, 10);
					InputFloat("##a", &robot_params[i].a);
					TableNextColumn();
					//TableSetupColumn("b", 0, 10);
					InputFloat("##b", &robot_params[i].d);
					TableNextColumn();
					//TableSetupColumn("c", 0, 10);
					InputFloat("##c", &robot_params[i].alpha);
					PopID();
				}
				EndTable();
			}
			if (Button("Apply")) {
				robot_->setDhParams(robot_params);
			}
			End();

			Begin("Performace");
			Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / GetIO().Framerate, GetIO().Framerate);
			End();
		}

		update(dt);
		render();
		
		// Swap the screen buffers
		glfwSwapBuffers(window_);
	}

	// imgui cleanup
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	// Terminate GLFW, clearing any resources allocated by GLFW.
	glfwTerminate();
}

void Application::update(float dt)
{
	if (running_ik_solution_)
	{
		updateJointControlSliders();
		// check if the robot reached the IK target already
		bool finished = (robot_->getJointAngles() - robot_->getTargetJointAngles()).norm() < 0.01;
		if (finished)
		{
			running_ik_solution_ = false;
		}
	}
	else
	{
		applyJointControls();
	}

	camera_.update(dt, window_);
	robot_->update(dt);
}

void Application::setIkTarget() {
	// convert mouse pos to normalized image coordinates
	Vector2d mouse_pos;
	int window_width, window_height;
	glfwGetCursorPos(window_, &mouse_pos.x(), &mouse_pos.y());
	glfwGetFramebufferSize(window_, &window_width, &window_height);
	float x = mouse_pos.x() / window_width;
	float y = mouse_pos.y() / window_height;
	x = x * 2.0f - 1.0f;
	y = 1.0f - (2.0f * y);

	Vector4f ray_clip = {x, y, -1.0f, 1.0f};
	Vector4f ray_eye = camera_.getProjectionMatrix().inverse() * ray_clip;
	ray_eye.z() = -1.0;
	ray_eye.w() = 0.0;

	Vector3f ray_world = (camera_.getViewMatrix().inverse() * ray_eye).head<3>();
	ray_world.normalize();

	Vector3f camera_position = camera_.getPosition();

	// ray should ge going from [camera_position] towards [ray_world]
	float t = 2;
	ray_start_ = camera_position;
	ray_end_ = camera_position + ray_world * t;

	robot_->setTargetTcpPosition(ray_end_);
}

void Application::abortRunningIkSolution() {
	// set robot target to its current angles
	robot_->setJointTargetAngles(robot_->getJointAngles());
	// return joint control to the sliders
	running_ik_solution_ = false;
}

void Application::updateJointControlSliders() {
	// set the joint slider values to actual joint angles
	for (int i = 0; i < joint_angle_controls_.size(); i++) {
		joint_angle_controls_[i] = robot_->getJointAngle(i);
	}
}

void Application::applyJointControls() {
	// set robot joint rotations to slider values
	for (int i = 0; i < joint_angle_controls_.size(); i++) {
		float curr = joint_angle_controls_[i];
		float prev = prev_controls_[i];
		if (abs(curr - prev) > 0.0001) {
			robot_->setJointTargetAngle(i, joint_angle_controls_[i]);
			prev_controls_[i] = curr;
		}
	}
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

	camera_.setAspectRatio(fAspect);

	checkGlErrors();

	// Set up transform matrices.
	// They will be fed to the shader program via uniform variables.

	// World space -> clip space transform: simple projection and camera.
	Matrix4f C = camera_.getViewMatrix();
	Matrix4f P = camera_.getProjectionMatrix();

	if (drawmode_ == DrawMode::MODE_MESH_WIREFRAME) {
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	}

	std::vector<Vertex> vertices = robot_->getMeshVertices();

	temp += 0.0001;
	Vector3f lightpos = Vector3f(6 * sin(temp), 3.5, 6 * cos(temp));
	BoxMesh lightmesh = BoxMesh(0.2, 0.2, 0.2, Vector4f(1, 1, 1, 1));
	lightmesh.transform(Translation3f(lightpos) * AngleAxisf::Identity());
	std::vector<Vertex> lightverts = lightmesh.getVertices();
	vertices.insert(vertices.end(), lightverts.begin(), lightverts.end());

	SphereMesh ikmesh = SphereMesh(0.01, Vector4f(1, 0.2, 0.2, 1.0));
	ikmesh.transform(Translation3f(ray_end_) * AngleAxisf::Identity());
	std::vector<Vertex> ikverts = ikmesh.getVertices();
	vertices.insert(vertices.end(), ikverts.begin(), ikverts.end());

	float square_size = 0.2;
	float grid_size = 50;

	std::vector<Vertex> planeVerts;

	for (int i = 0; i < grid_size; i++) {
		for (int j = 0; j < grid_size; j++) {
			Vector4f color = (i + j) % 2 == 0 ? Vector4f(0.8, 0.8, 0.8, 1.0) : Vector4f(0.3, 0.3, 0.3, 1.0);

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

	SphereMesh sphere(0.2f, { 0.0, 0.5, 0, 0.5});
	Affine3f t = AngleAxisf::Identity() * Translation3f(0, 0.2, 0);
	sphere.setToWorldTransform(t);
	std::vector<Vertex> sphere_verts = sphere.getVertices();
	vertices.insert(vertices.end(), sphere_verts.begin(), sphere_verts.end());

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	checkGlErrors();

	glBindBuffer(GL_ARRAY_BUFFER, gl_.simple_vertex_buffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * vertices.size(), vertices.data(), GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glUseProgram(gl_.simple_shader);

	Matrix4f model = Matrix4f::Identity();
	Vector3f viewpos = camera_.getPosition();

	glUniformMatrix4fv(gl_.model, 1, GL_FALSE, model.data());
	glUniformMatrix4fv(gl_.view, 1, GL_FALSE, C.data());
	glUniformMatrix4fv(gl_.projection, 1, GL_FALSE, P.data());

	glUniform3f(gl_.lightPos, lightpos.x(), lightpos.y(), lightpos.z());
	glUniform3f(gl_.viewPos, viewpos.x(), viewpos.y(), viewpos.z());
	glUniform3f(gl_.lightColor, 1, 1, 1);

	glBindVertexArray(gl_.simple_vao);
	glDrawArrays(GL_TRIANGLES, 0, (int)vertices.size());


	checkGlErrors();

	if (drawmode_ == DrawMode::MODE_MESH_WIREFRAME) {
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}

	glBindVertexArray(0);
	glUseProgram(0);

	// GUI rendering:
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

	// Check for OpenGL errors.
	checkGlErrors();
}
