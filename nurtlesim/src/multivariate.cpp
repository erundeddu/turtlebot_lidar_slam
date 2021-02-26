#include "nurtlesim/multivariate.hpp"
#include <armadillo>

Multivar::Multivar(arma::Mat<double> & cov)
{
	Q = cov;
	L = arma::chol(Q, "lower");
	for (std::size_t i = 0; i < cov.n_rows; ++i)
	{
		std::normal_distribution<> d(0.0, cov(i,i));
		dist_arr.push_back(d);
	}
}

// function from lecture notes
std::mt19937 & Multivar::get_random()
{
	// static variables inside a function are created once and persist for the remainder of the program
	static std::random_device rd{}; 
	static std::mt19937 mt{rd()};
	// we return a reference to the pseudo-random number genrator object. This is always the
	// same object every time get_random is called
	return mt;
}

std::vector<double> Multivar::draw()
{
	arma::Mat<double> u(Q.n_rows, 1);
	for (std::size_t i = 0; i < Q.n_rows; ++i)
	{
		u[i] = dist_arr[i](get_random());
	}
	return (arma::conv_to<std::vector<double>>::from(L*u));
}



