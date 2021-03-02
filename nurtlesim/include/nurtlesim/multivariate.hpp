#ifndef MULTIVARIATE_INCLUDE_GUARD_HPP
#define MULTIVARIATE_INCLUDE_GUARD_HPP
/// \file
/// \brief a class to store a multivariate zero-mean gaussian distribution

#include <armadillo>
#include <random>
#include <vector>

/// \brief a zero-mean multivariate Gaussian distribution
class Multivar
{
private:
	// distribution variance matrix
	arma::Mat<double> Q; 
	// lower Cholesky decomposition of the variance matrix Q 
	arma::Mat<double> L; 
	// array of one-variable normal distribution objects 
	std::vector<std::normal_distribution<>> dist_arr;
	
	std::mt19937 & get_random();

public:
	/// \brief constructs a zero-mean multivariate Gaussian distribution from a covariance matrix
	/// \param cov - the covariance matrix of the distribution
	Multivar(arma::Mat<double> & cov);
	
	/// \brief draws a vector from the multivariate Gaussian distribution that was initialized
	/// \return a vector of multivariate Gaussian random entries
	std::vector<double> draw();
};

#endif
