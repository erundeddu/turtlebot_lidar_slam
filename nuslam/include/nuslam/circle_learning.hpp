#ifndef CIRCLE_LEARNING_INCLUDE_GUARD_HPP
#define CIRCLE_LEARNING_INCLUDE_GUARD_HPP
/// \file
/// \brief functions to process laser data and fit circles through unsupervised and supervised learning

#include <vector>
#include "rigid2d/rigid2d.hpp"

namespace nuslam
{
	/// \brief structure to represent a circle on a plane
	struct Circle
	{
		// x coordinate of circle center
		double x = 0.0;
		// y coordinate of circle center
		double y = 0.0;
		// radius of circle
		double r = 1.0;
	};	
	
	/// \brief cluster an array of laser ranges given a distance threshold and discard clusters with a low number of elements
	/// \param ranges - array of range data
	/// \param thresh - threshold above which adjacent range data belong to different clusters
	/// \param min_n - minimum number of data points in a cluster to be retained
	/// \return a vector of vectors, where each vector represents a different cluster and contains the indices of the range data array belonging to the cluster
	std::vector<std::vector<int>> cluster_ranges(const std::vector<float> & ranges, double thresh, int min_n);
	
	/// \brief classifies a cluster of points into Circle or Not Circle
	/// \param ranges - array of range data
	/// \param cluster - array of indices relative to range data
	/// \param dtheta - angular increment between adjacent measurements
	/// \param min_mean - minimum value of angle mean (in radians) for Circle
	/// \param max_mean - maximum value of angle mean (in radians) for Circle
	/// \param max_std - maximum value of angle standard deviation (in radians) for Circle
	/// \return true if Circle, false if Not Circle
	bool is_circle(const std::vector<float> & ranges, const std::vector<int> & cluster, double dtheta, float min_mean, float max_mean, float max_std);
	
	/// \brief computes the mean of a vector
	/// \param v - a vector
	/// \return mean of v
	template <typename T>
	T mean(const std::vector<T> & v);
	
	/// \brief computes the standard deviation of a vector
	/// \param v - a vector
	/// \return standard deviation of v
	template <typename T>
	T stdev(const std::vector<T> & v);
	
	/// \brief converts range data in a cluster into relative xy coordinates
	/// \param ranges - array of range data
	/// \param cluster - array of indices relative to range data
	/// \param dtheta - angular increment between adjacent measurements
	/// \return vector of Vector2D representing relative xy coordinates of data points
	std::vector<rigid2d::Vector2D> range2xy(const std::vector<float> & ranges, const std::vector<int> & cluster, double dtheta);
	
	/// \brief fits a circle to data points
	/// \param pts - array of xy coordinates of data points
	/// \return fitted circle in center coordinates + radius form
	Circle fit_circle(const std::vector<rigid2d::Vector2D> & pts);
}

#endif
