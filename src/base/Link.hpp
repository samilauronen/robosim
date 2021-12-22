
#include "Robot.hpp"
namespace linkystuff {

// describes a link of a robot
struct Link {
	// DH parameters of this link
	DhParam p;

	// components of the link transformation matrix
	// calculated from the DH parameters of this link
	FW::Mat4f z_screw;
	FW::Mat4f x_screw;

	// combination of applying z_screw and  then x_screw
	// relates this link's frame to the frame of the previous link
	FW::Mat4f link_matrix;

	// transforms the coordinates of this link frame to world frame
	// created by multiplying together all link matrices of the previous links and the current link
	FW::Mat4f to_world;
};

// describes a combination of a joint and a link
struct JointedLink : Link {
	// rotation angle of the joint moving this link
	float rotation;
	// target rotation for the joint, current rotation will move towards this when update() is called
	float target_rotation;

	float joint_speed;
	bool is_moving;

	// used to evaluate the link matrix for any joint angle state
	// does not modify current state of the struct
	// used by iterative inverse kinematics solvers
	FW::Mat4f evalLinkMatrix(float rotationAngle) const;
	void updateLinkMatrix(float rotationAngle);
};

}