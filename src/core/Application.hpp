#pragma once

#include <vector>
#include <memory>

#include "Eigen/Dense"
#include "Robot.hpp"
#include "rendering/Camera.hpp"
#include "rendering/Shader.hpp"

#include "Event.hpp"


class Application
{
private:
	enum class DrawMode
	{
		MODE_MESH,
		MODE_MESH_WIREFRAME
	};

	struct glGeneratedIndices
	{
		// Shader programs
		GLuint simple_shader;

		// Vertex array objects
		GLuint simple_vao, point_vao;

		// Buffers
		GLuint simple_vertex_buffer;

		// shader uniforms
		GLint model, view, projection, lightPos, viewPos, lightColor, objectColor;
	};

public:
	Application();
	virtual         ~Application(void) {}

	void			createWindow(int width, int height);
	void			initRendering(void);
	void			run(void);
	void			render(void);
	void			update(float dt);
	void			applyJointControls();
	void			updateJointControlSliders();
	void			abortRunningIkSolution();
	void			mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
	void			setIkTarget();

	void			handleEvent(const Event& ev);

private:
	Application(const Application&); // forbid copy
	Application& operator=       (const Application&); // forbid assignment

private:
	Camera				camera_;
	GLFWwindow*			window_;
	DrawMode			drawmode_;
	bool				shading_toggle_;
	bool				shading_mode_changed_;
	std::vector<float>  joint_angle_controls_;
	std::vector<float>	prev_controls_;

	uint64_t time_end_;
	float temp = 0;
	bool running_ik_solution_ = false;

	float pid_p = 0.035;
	float pid_i = 0;
	float pid_d = 0;

	glGeneratedIndices	gl_;

	Eigen::Vector3f ray_start_, ray_end_;

	std::unique_ptr<Shader>	shader_;
	std::unique_ptr<Robot>  robot_;
};

