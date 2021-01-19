#include <iostream>
#include <ctype>
#include <cmath>
#include <string>
#include "rigid2d.hpp"

namespace rigid2d 
{

	std::ostream & operator<<(std::ostream & os, const Vector2D & v)
	{
		os << "[" << v.x << ", " << v.y << "]" << endl;
		return os;
	}

	std::istream & operator>>(std::istream & is, Vector2D & v)
	{
		char ch1 = is.peek();
		if (!(isdigit(ch1)))
		{
			ch1 = is.get();
		}
		is >> v.x;
		ch1 = is.peek();
		if (!(isdigit(ch1)))
		{
			ch1 = is.get();
		}
		is >> v.y;
		ch1 = is.peek();
		if (ch1 == ']')
		{
			ch1 = is.get();
		}
		return is;
		//TODO
	}
	
	Transform2D::Transform2D()
		: m_trans{struct Vector2D v}, m_radians{0.0}
	{
	}
	
	explicit Transform2D::Transform2D(const Vector2D & trans)
		: m_trans{trans}, m_radians{0.0}
	{
	}
	
	explicit Transform2D::Transform2D(double radians)
		: m_trans{Vector2D v}, m_radians{radians}
	{
	}
	
	Transform2D::Transform2D(const Vector2D & trans, double radians)
		: m_trans{trans}, m_radians{radians}
	{
	}
	
	Vector2D Transform2D::operator()(Vector2D v) const
	{
		Vector2D vout{v.x*cos(m_radians) - v.y*sin(m_radians) + m_trans.x, v.y*sin(m_radians) + v.y*cos(m_radians) + m_trans.y};
		return vout;
	}
	
	Transform2D Transform2D::inv() const
	{	
		Vector2D v{m_trans.x*cos(m_radians) + m_trans.y*sin(m_radians), m_trans.y*cos(m_radians) - m_trans.x*sin(m_radians)};
		Transform2D tf(v, -m_radians);
		return tf;
	}
	
	Transform2D & Transform::operator*=(const Transform2D & rhs)
	{
		m_radians += rhs.radians;
		m_trans.x += rhs.trans.x*cos(m_radians) - rhs.trans.y*sin(m_radians);
		m_trans.y += rhs.trans.x*sin(m_radians) + rhs.trans.y*cos(m_radians);
		return this;
	}
	
	std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
	{
		os << "dtheta (degrees): " << rad2deg(tf.radians) << " dx: " << tf.trans.x << " dy: " << tf.trans.y << endl;
		return os;
	}

	std::istream & operator>>(std::istream & is, Transform2D & tf)
	{
		char ch1 = is.peek;
		if (!(isdigit(ch1)))
		{
			std::string s;
			is >> s >> s >> deg2rad(tf.radians) >> s >> tf.trans.x >> s >> tf.trans.y;
		}
		else
		{
			is >> deg2rad(tf.radians) >> tf.trans.x >> tf.trans.y;
		}
		return is;
	}
	
	Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
	{
		lhs *= rhs;
		return lhs;
	}
		
}	

