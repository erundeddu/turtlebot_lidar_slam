#ifndef HELPER_INCLUDE_GUARD_HPP
#define HELPER_INCLUDE_GUARD_HPP
/// \file
/// \brief helper functions to perform calculations for slam

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <armadillo>
#include <cmath>

namespace nuslam 
{
	/// \brief compute the matrix A (derivative of state update with respect to the state)
	/// \param d - the diff drive robot at time t-1
	/// \param l_phi_wheel_new - the angle of the robot left wheel at time t
	/// \param r_phi_wheel_new - the angle of the robot right wheel at time t
	/// \param n - the number of landmarks
	/// \return the matrix of A as an armadillo matrix
	arma::Mat<double> compute_A_mat(rigid2d::DiffDrive & d, double l_phi_wheel_new, double r_phi_wheel_new, int n);
	
	/// \brief compute the matrix H_j
	/// \param dx - estimated relative x distance
	/// \param dy - estimated relative y distance
	/// \param n - the number of landmarks
	/// \param j - the current landmark index (1 - n)
	arma::Mat<double> compute_Hj_mat(double dx, double dy, int n, int j);
	
	/// \brief landmark information
    struct Landmark
    {
    	/// range measurement
        double r;
        /// bearing measurement
        double phi;  
        /// landmark id
        int id;
        
        /// \brief Construct a landmark
        /// \param r_arg - range measurement
        /// \param phi_arg - bearing measurement
        /// \param id_arg - landmark id
        Landmark(double r_arg, double phi_arg, int id_arg);
    };
    
    /// \brief initialize landmark in state estimate vector of the map
    /// param d - the diff drive robot at time t
    /// param lm - the landmark to initialize
    /// param M - the state estimate vector of the map at time t-1
    void initialize_landmark(rigid2d::DiffDrive & d, Landmark & lm, arma::Mat<double> & M);
    
    /// \brief compute theoretical measurement for a landmark
    /// param d - the diff drive robot at time t
    /// param M - the state estimate vector of the map at time t-1
    /// param id - the landmark id
    /// return theoretical measurement vector [r, phi]^T
    arma::Col<double> compute_meas(rigid2d::DiffDrive & d, arma::Mat<double> & M, double id);
}

#endif