#include "nuslam/circle_learning.hpp"
#include "rigid2d/rigid2d.hpp"
#include <vector>
#include <cmath>
#include <armadillo>

namespace nuslam 
{	
	std::vector<std::vector<int>> cluster_ranges(const std::vector<float> & ranges, double thresh, int min_n)
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
	
	bool is_circle(const std::vector<float> & ranges, const std::vector<int> & cluster, double dtheta, float min_mean, float max_mean, float max_std)
	{
		using namespace rigid2d;
		
		float r1 = ranges[cluster[0]];  // first range
		float r2 = ranges[cluster.back()];  // last range
		int n_data = cluster.size();
		Vector2D p1(r1, 0);  // first point (in relative reference frame)
		Vector2D p2(r2*cos(dtheta*(n_data-1)), r2*sin(dtheta*(n_data-1)));  // last point (in relative reference frame)
		std::vector<float> angles;
		for (int i=1; i<(n_data-2); ++i)
		{
			float ri = ranges[cluster[i]];
			Vector2D pi(ri*cos(dtheta*i), ri*sin(dtheta*i));
			Vector2D i1 = p1 - pi;
			Vector2D i2 = p2 - pi;
			angles.push_back(normalize_angular_difference(angle(i2), angle(i1)));
		}
		float angle_avg = mean(angles);
		float angle_std = stdev(angles);
		if ((angle_std <= max_std) && (angle_avg >= min_mean) && (angle_avg <= max_mean))
		{
			return true;
		}
		else
		{
			return false;
		}
	}	
	
	template <typename T>
	T mean(const std::vector<T> & v)
	{
		T sum = 0.0;
		for (std::size_t i=0; i<v.size(); ++i)
		{
			sum += v[i];
		}
		return (sum/v.size());
	}
	
	template <typename T>
	T stdev(const std::vector<T> & v)
	{
		T avg = mean(v);
		T ssq = 0.0;
		for (std::size_t i=0; i<v.size(); ++i)
		{
			ssq += ((v[i] - avg)*(v[i] - avg));
		}
		return (sqrt(ssq)/(v.size()-1));
	}
	
	std::vector<rigid2d::Vector2D> range2xy(const std::vector<float> & ranges, const std::vector<int> & cluster, double dtheta)
	{
		using namespace rigid2d;
		std::vector<rigid2d::Vector2D> v_xy;
		for (std::size_t i=0; i<cluster.size(); ++i)
		{
			Vector2D v(ranges[i]*cos(dtheta*i), ranges[i]*sin(dtheta*i));
			v_xy.push_back(v);
		}
		return v_xy;
	}
	
	Circle fit_circle(const std::vector<rigid2d::Vector2D> & pts)
	{
		using namespace rigid2d;
		
		Vector2D avg_pt;
		for (std::size_t i=0; i<pts.size(); ++i)
		{
			avg_pt += pts[i];
		}
		avg_pt *= (1.0/pts.size());
		
		std::vector<double> x;
		std::vector<double> y;
		std::vector<double> z;
		for (std::size_t i=0; i<pts.size(); ++i)
		{
			double xi = pts[i].x-avg_pt.x;
			double yi = pts[i].y-avg_pt.y;
			double zi = xi*xi + yi*yi;
			x.push_back(xi);
			y.push_back(yi);
			z.push_back(zi);
		}	
		double z_avg = mean(z);
		
		arma::Col<double> xcol(x);
		arma::Col<double> ycol(y);
		arma::Col<double> zcol(z);
		arma::Mat<double> Z = arma::join_rows(zcol,xcol,ycol,arma::ones(pts.size(),1));
	}
}
