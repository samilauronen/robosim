#include <array>

#include "base/Math.hpp"

namespace InverseKinematics {
	// base class for IK solvers for robots with any number of joints
	template <unsigned N_JOINTS>
	class IKSolver {
		// takes in desired position and orientation of TCP frame, returns joint angles to achieve that
		virtual std::array<float, N_JOINTS> solve(FW::Vec3f location, FW::Vec3f orientation) = 0;
	};

	// IK solver for the UR5 robot with six joints
	class UR5_IKSolver : IKSolver<6> {

	};

	// TODO: Add numerical IK solvers based on the paper recommendations
	// http://math.ucsd.edu/~sbuss/ResearchWeb/ikmethods/iksurvey.pdf
	// Especially Jacobian Transpose is said to be easy to implments, fast and accurate. Try that.
}