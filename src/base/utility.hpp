#pragma once

#include "../framework/base/Math.hpp"

#include <iostream>
#include <chrono>

template <typename T>
void printVec3(const T& t) {
	cout << "(" << t.x << " " << t.y << " " << t.z << ")" << endl;
}

// Construct a matrix from the columns [a, b, c, d].
inline FW::Mat4f makeMat4f(const FW::Vec4f& a, const FW::Vec4f& b, const FW::Vec4f& c, const FW::Vec4f& d) {
	FW::Mat4f A;
	A.col(0) = a;
	A.col(1) = b;
	A.col(2) = c;
	A.col(3) = d;
	return A;
}

inline FW::Mat4f combineToMat4f(const FW::Mat3f& rotation, const FW::Vec3f& translation) {
	FW::Mat4f result;
	result.setCol(0, FW::Vec4f(rotation.getCol(0), 0));
	result.setCol(1, FW::Vec4f(rotation.getCol(1), 0));
	result.setCol(2, FW::Vec4f(rotation.getCol(2), 0));
	result.setCol(3, FW::Vec4f(translation, 1));
	return result;
}


inline uint64_t currentTimeMillis() {
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::
		now().time_since_epoch()).count();
}

inline uint64_t currentTimeMicros() {
	return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::
		now().time_since_epoch()).count();
}

// http://stackoverflow.com/questions/2270726/how-to-determine-the-size-of-an-array-of-strings-in-c
template <typename T, std::size_t N>
char (&static_sizeof_array( T(&)[N] ))[N];   // declared, not defined
#define SIZEOF_ARRAY( x ) sizeof(static_sizeof_array(x))
