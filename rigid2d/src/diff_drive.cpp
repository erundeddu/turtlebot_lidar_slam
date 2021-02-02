#include "rigid2d/diff_drive.hpp"

namespace rigid2d
{
	DiffDrive::DiffDrive(RobotPose q, double wheel_base, double wheel_radius, double r_wheel_phi, double l_wheel_phi)
		: m_q(q)
		, m_wheel_base(wheel_base)
		, m_wheel_radius(wheel_radius)
		, m_r_wheel_phi(r_wheel_phi)
		, m_l_wheel_phi(l_wheel_phi)
	{
	}
	
	DiffDrive::DiffDrive(RobotPose q, double wheel_base, double wheel_radius)
		: m_q(q)
		, m_wheel_base(wheel_base)
		, m_wheel_radius(wheel_radius)
		, m_r_wheel_phi(0.0)
		, m_l_wheel_phi(0.0)
	{
	}
	
	DiffDrive::DiffDrive(double wheel_base, double wheel_radius)
		: m_wheel_base(wheel_base)
		, m_wheel_radius(wheel_radius)
		, m_r_wheel_phi(0.0)
		, m_l_wheel_phi(0.0)
	{
	}
	
	DiffDrive::DiffDrive()
		: m_r_wheel_phi(0.0)
		, m_l_wheel_phi(0.0)
	{
	}
	
	void DiffDrive::setPhysicalParams(double wheel_base, double wheel_radius)
	{
		m_wheel_base = wheel_base;
		m_wheel_radius = wheel_radius;
	}
	
	void DiffDrive::updatePose(double r_wheel_phi_new, double l_wheel_phi_new)
	{
		// Determine wheel angle change
		double d_phi_r = r_wheel_phi_new - m_r_wheel_phi;
		double d_phi_l = l_wheel_phi_new - m_l_wheel_phi;
		// Update absolute wheel angles
		m_r_wheel_phi += d_phi_r;
		m_l_wheel_phi += d_phi_l;
		// Compute body twist
		double d_theta_b = (d_phi_r - d_phi_l) * m_wheel_radius / m_wheel_base;
		double d_x_b = (d_phi_r + d_phi_l) * m_wheel_radius / 2;
		double d_y_b = 0.0;
		Vector2D d_v(d_x_b, d_y_b);
		Twist2D d_q_b(d_v, d_theta_b);  
		Vector2D v;  // zero vector
		Transform2D t_a(v, m_q.theta);  // frame aligned with the world but located at the body
		Twist2D d_q = t_a.change_twist_frame(d_q_b);
		// Update pose
		m_q.theta += d_q.getW();
		m_q.x += d_q.getVx();
		m_q.y += d_q.getVy();
	}
	
	WheelVel DiffDrive::twist2WheelVel(Twist2D & tw) const
	{
		WheelVel wv;
		wv.r_vel = (2*tw.getVx() + tw.getW()*m_wheel_base)/(2.0*m_wheel_radius);
		wv.l_vel = (2*tw.getVx() - tw.getW()*m_wheel_base)/(2.0*m_wheel_radius);
		return wv;
	}
	
	double DiffDrive::getTheta() const
	{
		return m_q.theta;
	}
		
	double DiffDrive::getX() const
	{
		return m_q.x;
	}
	
	double DiffDrive::getY() const
	{
		return m_q.y;
	}
	
	double DiffDrive::getRWheelPhi() const
	{
		return m_r_wheel_phi;
	}
	
	double DiffDrive::getLWheelPhi() const
	{
		return m_l_wheel_phi;
	}
}
