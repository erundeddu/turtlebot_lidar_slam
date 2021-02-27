#ifndef HELPER_INCLUDE_GUARD_HPP
#define HELPER_INCLUDE_GUARD_HPP
/// \file
/// \brief helper functions to perform calculations for slam

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <armadillo>

namespace nuslam {

	/// \brief compute the matrix A (derivative of state update with respect to the state)
	/// \param d - the diff drive robot at time t-1
	/// \param l_phi_wheel_new - the angle of the robot left wheel at time t
	/// \param r_phi_wheel_new - the angle of the robot right wheel at time t
	/// \param n - the number of landmarks
	/// \return the matrix of A as an armadillo matrix
	arma::Mat<double> compute_A_mat(rigid2d::DiffDrive & d, double l_phi_wheel_new, double r_phi_wheel_new, int n);
}

#endif
