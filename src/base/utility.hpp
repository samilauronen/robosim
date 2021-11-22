#pragma once

#include "../framework/base/Math.hpp"

#include <iostream>

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

// http://stackoverflow.com/questions/2270726/how-to-determine-the-size-of-an-array-of-strings-in-c
template <typename T, std::size_t N>
char (&static_sizeof_array( T(&)[N] ))[N];   // declared, not defined
#define SIZEOF_ARRAY( x ) sizeof(static_sizeof_array(x))
