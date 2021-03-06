#include "nuslam/circle_learning.hpp"
#include <vector>
#include <cmath>

namespace nuslam 
{	
	std::vector<std::vector<int>> cluster_ranges(std::vector<float> ranges, double thresh, int min_n)
	{
		std::vector<std::vector<int>> clusters_list;
		std::vector<int> cluster;
		int num_measurements = ranges.size();
		for (int i=0; i<num_measurements; ++i)
		{	
			if (cluster.empty())
			{
				cluster.push_back(i);
			}
			else
			{
				if (std::abs(ranges[cluster.back()] - ranges[i]) < thresh)
				{
					cluster.push_back(i);
				}
				else
				{
					clusters_list.push_back(cluster);
					cluster.clear();
					cluster.push_back(i);
				}
			}
			if (i == num_measurements - 1) // last element
			{
				if (std::abs(ranges[0] - ranges[i]) < thresh)
				{
					for (std::size_t j=0; j<cluster.size(); ++j)
					{
						clusters_list[0].push_back(cluster[j]);
					}
				}
				else
				{
					clusters_list.push_back(cluster);
				}
			}
		}
		
		for (std::size_t k=0; k<clusters_list.size(); ++k)
		{
			if ((int)clusters_list[k].size() < min_n)
			{
				clusters_list.erase(clusters_list.begin()+k);
				k -= 1;
			}
		}
		return clusters_list;
	}	
}
