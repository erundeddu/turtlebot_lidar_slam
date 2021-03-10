#ifndef CIRCLES_INCLUDE_GUARD_HPP
#define CIRCLES_INCLUDE_GUARD_HPP
/// \file
/// \brief geometric intersection calculations between lines and circles

#include "rigid2d/rigid2d.hpp"

namespace circles
{
	/// \brief a structure to be returned by functions to get if intersection exists, and which point (if any) the intersection is
	struct Intersection
	{
		/// if true, an intersection exists
		bool is_intersection = false;
		/// x coordinate of the intersection
		double x = 0.0;
		/// y coordinate of the intersection
		double y = 0.0;
	}; 

	/// \brief returns information about the intersection between the line formed by two points and a circle
	/// \param p1 - the first point defining the line
	/// \param p2 - the second point defining the line
	/// \param p_circle - the center of the circle
	/// \param radius - the radius of the circle
	/// \return whether the circle intersects the line p1-p2, and, if so, the intersection point closest to p1 in the direction p1-p2
	Intersection compute_intersection(rigid2d::Vector2D p1, rigid2d::Vector2D p2, rigid2d::Vector2D p_circle, double radius);
	
	/// \brief returns sign (+1/-1) of a number
	/// \param x - a number
	/// \return -1 if x<0, else 1
	int sgn(double x);
}
#endif
