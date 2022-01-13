#pragma once

#include "Eigen/Dense"

#include <iostream>
#include <chrono>

// GLEW
#define GLEW_STATIC
#include <GL/glew.h>

// GLFW
#include <GLFW/glfw3.h>


inline uint64_t currentTimeMillis() {
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::
		now().time_since_epoch()).count();
}

inline uint64_t currentTimeMicros() {
	return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::
		now().time_since_epoch()).count();
}

// TODO: could be moved to a GraphicsUtils file?
inline void drawFrame(Eigen::Affine3f world_to_frame, float scale) {
	Eigen::Vector3f origin = world_to_frame.translation();
	Eigen::Matrix3f orientation = world_to_frame.rotation();
	Eigen::Vector3f i, j, k;
	i = orientation.col(0) * scale;
	j = orientation.col(1) * scale;
	k = orientation.col(2) * scale;

	glBegin(GL_LINES);
	glColor3f(1, 0, 0);
	glVertex3f(origin.x(), origin.y(), origin.z());
	glVertex3f(origin.x() + i.x(), origin.y() + i.y(), origin.z() + i.z());
	glColor3f(0, 1, 0); // green
	glVertex3f(origin.x(), origin.y(), origin.z());
	glVertex3f(origin.x()+ j.x(), origin.y() + j.y(), origin.z() + j.z());
	glColor3f(0, 0, 1); // blue
	glVertex3f(origin.x(), origin.y(), origin.z());
	glVertex3f(origin.x() + k.x(), origin.y()+ k.y(), origin.z() + k.z());
	glEnd();
}

inline void checkGlErrors() {
	GLenum err;
	while ((err = glGetError()) != GL_NO_ERROR) { std::cerr << err << std::endl; abort(); };
}

// http://stackoverflow.com/questions/2270726/how-to-determine-the-size-of-an-array-of-strings-in-c
template <typename T, std::size_t N>
char (&static_sizeof_array( T(&)[N] ))[N];   // declared, not defined
#define SIZEOF_ARRAY( x ) sizeof(static_sizeof_array(x))
