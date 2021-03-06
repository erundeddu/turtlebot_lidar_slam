#ifndef CIRCLE_LEARNING_INCLUDE_GUARD_HPP
#define CIRCLE_LEARNING_INCLUDE_GUARD_HPP
/// \file
/// \brief functions to process laser data and fit circles through unsupervised and supervised learning

#include <vector>

namespace nuslam
{
	/// \brief cluster an array of laser ranges given a distance threshold and discard clusters with a low number of elements
	/// \param ranges - array of range data
	/// \param thresh - threshold above which adjacent range data belong to different clusters
	/// \param min_n - minimum number of data points in a cluster to be retained
	/// \return a vector of vectors, where each vector represents a different cluster and contains the indices of the range data array belonging to the cluster
	std::vector<std::vector<double>> cluster_ranges(double ranges[], double thresh, int min_n);
}

#endif
