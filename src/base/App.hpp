#pragma once

#include "robot.hpp"

#include "gui/Window.hpp"
#include "gui/CommonControls.hpp"
#include "3d/CameraControls.hpp"

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

	// simple_shader uniforms
	GLint simple_world_to_clip_uniform, simple_shading_mix_uniform;

	// ssd_shader uniforms
	GLint ssd_world_to_clip_uniform, ssd_shading_mix_uniform, ssd_transforms_uniform;
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
	std::vector<Vec3f>	joint_colors_;
	std::vector<float>  joint_angle_controls_;

	glGeneratedIndices	gl_;

	std::unique_ptr<Robot>	rob_;
};

} // namespace FW
