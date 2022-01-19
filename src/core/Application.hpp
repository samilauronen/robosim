#pragma once

#include <vector>
#include <memory>

#include "Eigen/Dense"
#include "Robot.hpp"
#include "rendering/Camera.hpp"
#include "rendering/Shader.hpp"


class Application
{
private:
	enum class DrawMode
	{
		MODE_SKELETON,
		MODE_MESH_CPU,
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
	bool ik_ = true;

	glGeneratedIndices	gl_;

	Eigen::Vector3f ray_start_, ray_end_;

	std::unique_ptr<Shader>	shader_;
	std::unique_ptr<Robot>  robot_;
};

