#ifndef CIRCLE_LEARNING_INCLUDE_GUARD_HPP
#define CIRCLE_LEARNING_INCLUDE_GUARD_HPP
/// \file
/// \brief functions to process laser data and fit circles through unsupervised and supervised learning

#include <vector>
#include "rigid2d/rigid2d.hpp"

namespace nuslam
{
	/// \brief cluster an array of laser ranges given a distance threshold and discard clusters with a low number of elements
	/// \param ranges - array of range data
	/// \param thresh - threshold above which adjacent range data belong to different clusters
	/// \param min_n - minimum number of data points in a cluster to be retained
	/// \return a vector of vectors, where each vector represents a different cluster and contains the indices of the range data array belonging to the cluster
	std::vector<std::vector<int>> cluster_ranges(std::vector<float> & ranges, double thresh, int min_n);
	
	/// \brief classifies a cluster of points into Circle or Not Circle
	/// \param ranges - array of range data
	/// \param cluster - array of indices relative to range data
	/// \param dtheta - angular increment between adjacent measurements
	/// \param min_mean - minimum value of angle mean (in radians) for Circle
	/// \param max_mean - maximum value of angle mean (in radians) for Circle
	/// \param max_std - maximum value of angle standard deviation (in radians) for Circle
	/// \return true if Circle, false if Not Circle
	bool is_circle(std::vector<float> & ranges, std::vector<int> & cluster, double dtheta, float min_mean, float max_mean, float max_std);
	
	/// \brief computes the mean of a vector of float
	/// \param v - a vector of float
	/// \return mean of v
	float mean(std::vector<float> & v);
	
	/// \brief computes the standard deviation of a vector of float
	/// \param v - a vector of float
	/// \return standard deviation of v
	float stdev(std::vector<float> & v);
	
	/// \brief converts range data in a cluster into relative xy coordinates
	/// \param ranges - array of range data
	/// \param cluster - array of indices relative to range data
	/// \param dtheta - angular increment between adjacent measurements
	/// \return vector of Vector2D representing relative xy coordinates of data points
	std::vector<rigid2d::Vector2D> range2xy(std::vector<float> & ranges, std::vector<int> & cluster, double dtheta);
}

#endif
