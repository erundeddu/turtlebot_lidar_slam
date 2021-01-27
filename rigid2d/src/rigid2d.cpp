#include<iostream>
#include<sstream>
#include<cctype>
#include<cmath>
#include<string>
#include "rigid2d/rigid2d.hpp"

namespace rigid2d 
{

	std::ostream & operator<<(std::ostream & os, const Vector2D & v)
	{
		os << "[" << v.x << ", " << v.y << "]" << std::endl;
		return os;
	}

	std::istream & operator>>(std::istream & is, Vector2D & v)
	{	
		char ch = is.peek();
		while ((ch == ' ') || (ch == '\n'))
		{
			ch = is.get();
			ch = is.peek();
		}
		ch = is.peek();
		if (ch == '[')
		{
			ch = is.get();  // eliminate [
			is >> v.x;
			ch = is.get();  // eliminate ,
			is >> v.y; 
			ch = is.get();  // eliminate ]
		}
		else
		{
			is >> v.x >> v.y;
		}
		return is;
	}
	
	Vector2D & Vector2D::normalize()
	{
		double mag = sqrt(x*x + y*y);
		if (!(almost_equal(mag, 0.0)))
		{
			x *= (1/mag);
			y *= (1/mag);
		}	
		return *this;
	}
	
	Transform2D::Transform2D()
		: m_trans{}
		, m_radians{0.0}
	{
	}
	
	Transform2D::Transform2D(const Vector2D & trans)
		: m_trans{trans}
		, m_radians{0.0}
	{
	}
	
	Transform2D::Transform2D(double radians)
		: m_trans{}
		, m_radians{radians}
	{
	}
	
	Transform2D::Transform2D(const Vector2D & trans, double radians)
		: m_trans{trans}
		, m_radians{radians}
	{
	}
	
	double Transform2D::getX() const
	{
		return m_trans.x;
	}
        
    double Transform2D::getY() const
    {
		return m_trans.y;
	}
    
    double Transform2D::getStheta() const
    {
		return sin(m_radians);
	}
    
    double Transform2D::getCtheta() const
    {
		return cos(m_radians);
	}
	
	Vector2D Transform2D::operator()(Vector2D v) const
	{
		Vector2D vout{v.x*cos(m_radians) - v.y*sin(m_radians) + m_trans.x, v.y*cos(m_radians) + v.x*sin(m_radians) + m_trans.y};
		return vout;
	}
	
	Transform2D Transform2D::inv() const
	{	
		Vector2D v{-m_trans.x*cos(m_radians) - m_trans.y*sin(m_radians), - m_trans.y*cos(m_radians) + m_trans.x*sin(m_radians)};
		Transform2D tf(v, -m_radians);
		return tf;
	}
	
	Twist2D Transform2D::change_twist_frame(const Twist2D & tw) const
	{
		Vector2D v{tw.m_rot_v*m_trans.y + tw.m_trans_v.x*cos(m_radians) - tw.m_trans_v.y*sin(m_radians), -tw.m_rot_v*m_trans.x + tw.m_trans_v.x*sin(m_radians) + tw.m_trans_v.y*cos(m_radians)};
		Twist2D twout(v, tw.m_rot_v);
		return twout;
	} 
	
	Transform2D & Transform2D::operator*=(const Transform2D & rhs)
	{
		m_trans.x += rhs.m_trans.x*cos(m_radians) - rhs.m_trans.y*sin(m_radians);
		m_trans.y += rhs.m_trans.x*sin(m_radians) + rhs.m_trans.y*cos(m_radians);
		m_radians += rhs.m_radians;
		return *this;
	}
	
	std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
	{
		os << "dtheta (degrees): " << rad2deg(tf.m_radians) << " dx: " << tf.m_trans.x << " dy: " << tf.m_trans.y << std::endl;
		return os;
	}

	std::istream & operator>>(std::istream & is, Transform2D & tf)
	{
		char ch = is.peek();
		while ((ch == ' ') || (ch == '\n'))
		{
			ch = is.get();
			ch = is.peek();
		}
		ch = is.peek();
		std::string str;
		Vector2D trans;
		double degrees;
		if (ch == 'd')  // cout format
		{
			is >> str >> str >> degrees >> str >> trans.x >> str >> trans.y;
		}
		else  // space separated format
		{
			is >> degrees >> trans.x >> trans.y;
		}
		Transform2D tnew(trans, deg2rad(degrees));
		tf = tnew;
		return is;
	}
	
	Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
	{
		lhs *= rhs;
		return lhs;
	}
	
	Twist2D::Twist2D()
		: m_trans_v{}
		, m_rot_v{0.0}
	{
	}
	
	Twist2D::Twist2D(const Vector2D & trans_v)
		: m_trans_v{trans_v}
		, m_rot_v{0.0}
	{
	}
	
	Twist2D::Twist2D(double rot_v)
		: m_trans_v{}
		, m_rot_v{rot_v}
	{
	}
	
	Twist2D::Twist2D(const Vector2D & trans_v, double rot_v)
		: m_trans_v{trans_v}
		, m_rot_v{rot_v}
	{
	}
	
	double Twist2D::getVx() const
	{
		return m_trans_v.x;
	}
        
    double Twist2D::getVy() const
    {
		return m_trans_v.y;
	}
	
	double Twist2D::getW() const
    {
		return m_rot_v;
	}
	
	std::ostream & operator<<(std::ostream & os, const Twist2D & tw)
	{
		os << "w (radians/time): " << tw.m_rot_v << " x_dot: " << tw.m_trans_v.x << " y_dot: " << tw.m_trans_v.y << std::endl;
		return os;
	}
	
	std::istream & operator>>(std::istream & is, Twist2D & tw)
	{
		char ch = is.peek();
		while ((ch == ' ') || (ch == '\n'))
		{
			ch = is.get();
			ch = is.peek();
		}
		ch = is.peek();
		std::string str;
		Vector2D trans_v;
		double rot_v;
		if (ch == 'w')  // cout format
		{
			is >> str >> str >> rot_v >> str >> trans_v.x >> str >> trans_v.y;
		}
		else  // space separated format
		{
			is >> rot_v >> trans_v.x >> trans_v.y;
		}
		Twist2D twnew(trans_v, rot_v);
		tw = twnew;
		return is;
	}

		
}	

