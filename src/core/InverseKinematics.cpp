#include "InverseKinematics.hpp"

namespace IK {


using namespace Eigen;

IKSolution SimpleIKSolver::solve(const Robot& robot, const Vector3f& tcp_target_wrt_world) {
	Affine3f base_to_world = robot.getWorldToBase().inverse();
	Vector3f target_wrt_base = base_to_world * tcp_target_wrt_world;

	// updated every iteration
	Eigen::VectorXf solutionJointAngles = robot.getJointAngles();

	// current position and it's difference from target
	Vector3f current_wrt_base = base_to_world * robot.getTcpWorldPosition();
	Vector3f delta_p = target_wrt_base - current_wrt_base;

	uint64_t start_time = currentTimeMicros();

	// still too far away and have time?
	while (delta_p.norm() > DISTANCE_THRESHOLD && (currentTimeMicros() - start_time) < TIMEOUT_MICROS) {
		// difference vec from target position
		Vector<float, 6> dp;
		dp << delta_p.x(), delta_p.y(), delta_p.z(), 0, 0, 0;

		MatrixXf jacobian = robot.getJacobian(solutionJointAngles);

		// equation of form J * d_theta = dp
		// where J is jacobian, d_theta is difference in joint angles, dp is difference in TCP position
		// solve for d_theta
		VectorXf delta_theta = jacobian.colPivHouseholderQr().solve(dp);

		// clamp results, no crazy joint angles
		for (int i = 0; i < delta_theta.rows(); i++) {
			if (delta_theta(i) > FW_PI / 8) delta_theta(i) = FW_PI / 8;
			if (delta_theta(i) < -FW_PI / 8) delta_theta(i) = -FW_PI / 8;
		}

		// update solution
		solutionJointAngles += delta_theta;

		// where are we at with the current solution?
		current_wrt_base = base_to_world * robot.getTcpWorldPosition(solutionJointAngles);
		delta_p = target_wrt_base - current_wrt_base;
	}
	int time_taken = currentTimeMicros() - start_time;
	bool timed_out = time_taken >= TIMEOUT_MICROS;

	IKSolution solution;
	solution.joint_angles = solutionJointAngles;
	solution.time_taken_micros = time_taken;
	solution.timed_out = timed_out;

	return solution;
}



} // namespace IK