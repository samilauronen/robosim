#pragma once

#include "core/Robot.hpp"

#include "gui/Window.hpp"
#include "gui/CommonControls.hpp"
#include "3d/CameraControls.hpp"
#include "Eigen/Dense"

#include <vector>
#include <memory>

namespace FW {

struct glGeneratedIndices
{
	// Shader programs
	GLuint simple_shader, ssd_shader;

	// Vertex array objects
	GLuint simple_vao, ssd_vao, point_vao;

	// Buffers
	GLuint simple_vertex_buffer, ssd_vertex_buffer;

	// shader uniforms
	GLint model, view, projection, lightPos, viewPos, lightColor, objectColor;
};

class App : public Window::Listener
{
private:
	enum DrawMode
	{
		MODE_SKELETON,
		MODE_MESH_CPU,
		MODE_MESH_WIREFRAME
	};

public:
					App             (void);
	virtual         ~App            (void) {}

	virtual bool    handleEvent     (const Window::Event& ev);

private:
	void			initRendering		(void);
	void			render				(void);
	void			renderSkeleton		(void);

	
private:
					App             (const App&); // forbid copy
	App&            operator=       (const App&); // forbid assignment

private:
	Window			window_;
	CommonControls	common_ctrl_;
	CameraControls  camera_ctrl_;

	DrawMode			drawmode_;
	bool				shading_toggle_;
	bool				shading_mode_changed_;
	std::vector<float>  joint_angle_controls_;
	std::vector<float>	prev_controls_;

	uint64_t time_end_;
	float temp = 0;
	bool ik_ = true;

	glGeneratedIndices	gl_;

	FW::Vec3f ray_start_, ray_end_;

	std::unique_ptr<Robot> rob_;
};

} // namespace FW
