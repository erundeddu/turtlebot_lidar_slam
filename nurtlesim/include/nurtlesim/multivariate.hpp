#ifndef MULTIVARIATE_INCLUDE_GUARD_HPP
#define MULTIVARIATE_INCLUDE_GUARD_HPP
/// \file
/// \brief a class to store a multivariate zero-mean gaussian distribution

#include <armadillo>
#include <random>
#include <vector>

class Multivar
{
private:
	arma::Mat<double> Q;
	arma::Mat<double> L;
	std::vector<std::normal_distribution<>> dist_arr;
	
	std::mt19937 & get_random();

public:
	Multivar(arma::Mat<double> & cov);
	
	std::vector<double> draw();
};

#endif
