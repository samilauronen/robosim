#include "Link.hpp"
#include "utility.hpp"

namespace linkystuff {

FW::Mat4f JointedLink::evalLinkMatrix(float rotationAngle) const
{
	// https://www.youtube.com/watch?v=nuB_7BkYNMk
	Vec3f transl = Vec3f(0, 0, p.d);						// first move up so that we are at the level of the common normal
	Mat3f rot = Mat3f::rotation(Vec3f(0, 0, 1), rotationAngle);	// then rotate around z to align x with next x

	Mat4f z_screw = combineToMat4f(rot, transl);  // translation and rotation with respect to z

	// after applying the z-screw, we are at the correct level and our x-axis points in the correct direction
	// now move towards our new x by link length
	transl = Vec3f(p.a, 0, 0);
	rot = Mat3f::rotation(Vec3f(1, 0, 0), p.alpha);	// apply link twist (rotate around x-axis)

	Mat4f x_screw = combineToMat4f(rot, transl);	// translation and rotation with respect to x

	return z_screw * x_screw;
}

void JointedLink::updateLinkMatrix(float rotationAngle) {
	this->link_matrix = this->evalLinkMatrix(rotationAngle);
}

}