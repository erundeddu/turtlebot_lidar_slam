#include "nuslam/helper.hpp"
#include <armadillo>
#include <cmath>

namespace nuslam
{
	arma::Mat<double> compute_A_mat(rigid2d::DiffDrive & d, double l_phi_wheel_new, double r_phi_wheel_new, int n)
	{
		using rigid2d;
		
		Twist2D t = d.getBodyTwist(l_phi_wheel_new, r_phi_wheel_new, 1.0);
		arma::Mat<double> A_sub(3, 3, arma::fill::zeros);
		double dx = t.getVx();
		double dth = t.getW();
		double theta = d.getTheta();
		if (almost_equal(dth, 0.0))  // case 1: d(theta)_t == 0
		{
			A_sub(1,0) = -dx*sin(theta);
			A_sub(2,0) = dx*cos(theta);
		}
		else  // case 2: d(theta)_t != 0
		{
			A_sub(1,0) = (cos(theta + dth) - cos(theta))*dx/dth;
			A_sub(2,0) = (sin(theta + dth) - sin(theta))*dx/dth;
		}
		arma::Mat<double> A = arma::join_cols(arma::join_rows(A_sub, arma::zeros(3,2*n)), arma::join_rows(arma::zeros(2*n,3),arma::zeros(2*n,2*n)));
		A += arma::eye(3+2*n,3+2*n);
		return A;
	}

	arma::Mat<double> compute_Hj_mat(double dx, double dy, int n, int j)
	{
		double d = dx*dx + dy*dy;
		arma::Mat<double> h1 = {{0, -dx/sqrt(d), -dy/sqrt(d)},
								{-1, dy/d, -dx/d}};
		arma::Mat<double> h2 = arma::zeros(2,2*(j-1));
		arma::Mat<double> h3 = {{dx/sqrt(d), dy/sqrt(d)},
								{-dy/d, dx/d}};
		arma::Mat<double> h4 = arma::zeros(2,2*(n-j));
		arma::Mat<double> H = join_rows(h1,h2,h3,h4);
		return H;
	}
}
