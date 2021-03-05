#include "nurtlesim/circles.hpp"
#include "rigid2d/rigid2d.hpp"
#include <cmath>

namespace circles
{
	Intersection compute_intersection(rigid2d::Vector2D p1, rigid2d::Vector2D p2, rigid2d::Vector2D p_circle, double radius)
	{
		using namespace rigid2d;
		// make p1, p2 relative to the center of the circle
		p1.x -= p_circle.x;
		p2.x -= p_circle.x;
		p1.y -= p_circle.y;
		p2.y -= p_circle.y;
		// using algorithm described here: https://mathworld.wolfram.com/Circle-LineIntersection.html
		double dx = p2.x - p1.x;
		double dy = p2.y - p1.y;
		double dr = sqrt(dx*dx + dy*dy);
		double D = p1.x*p2.y - p2.x*p1.y;
		double discriminant = radius*radius*dr*dr - D*D;
		Intersection i1;  // first possible intersection
		Intersection i2;  // second possible intersection
		if (discriminant < 0)
		{
			i1.is_intersection = false;
			return i1;
		}
		else
		{	
			i1.is_intersection = true;
			i2.is_intersection = true;
			i1.x = (D*dy+sgn(dy)*dx*sqrt(discriminant))/(dr*dr);
			i2.x = (D*dy-sgn(dy)*dx*sqrt(discriminant))/(dr*dr);
			i1.y = (-D*dx+std::abs(dy)*sqrt(discriminant))/(dr*dr);
			i2.y = (-D*dx-std::abs(dy)*sqrt(discriminant))/(dr*dr);
			Vector2D p1_i1(p1.x-i1.x, p1.y-i1.y);
			Vector2D p1_i2(p1.x-i2.x, p1.y-i2.y);
			if (magnitude(p1_i1) < magnitude(p1_i2))
			{
				// translate the intersection point back to the world frame
				i1.x += p_circle.x;
				i1.y += p_circle.y;
				return i1;
			}
			else
			{
				// translate the intersection point back to the world frame
				i2.x += p_circle.x;
				i2.y += p_circle.y;
				return i2;
			}
		}
	}
	
	int sgn(double x)
	{
		if (x < 0)
		{
			return -1;
		}
		else
		{
			return 1;
		}
	}
}
