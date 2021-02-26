#include "rigid2d/diff_drive.hpp"

namespace rigid2d
{
	DiffDrive::DiffDrive(double wheel_base, double wheel_radius, double l_wheel_phi, double r_wheel_phi)
		: m_wheel_base(wheel_base)
		, m_wheel_radius(wheel_radius)
		, m_l_wheel_phi(l_wheel_phi)
		, m_r_wheel_phi(r_wheel_phi)
	{
	}
	
	DiffDrive::DiffDrive(double wheel_base, double wheel_radius)
		: m_wheel_base(wheel_base)
		, m_wheel_radius(wheel_radius)
		, m_l_wheel_phi(0.0)
		, m_r_wheel_phi(0.0)
	{
	}
	
	DiffDrive::DiffDrive()
		: m_l_wheel_phi(0.0)
		, m_r_wheel_phi(0.0)
	{
	}
	
	void DiffDrive::setPhysicalParams(double wheel_base, double wheel_radius)
	{
		m_wheel_base = wheel_base;
		m_wheel_radius = wheel_radius;
	}
	
	void DiffDrive::updatePose(double l_wheel_phi_new, double r_wheel_phi_new, double l_slip, double r_slip)
	{
		// Determine wheel angle change
		double d_phi_l = normalize_angular_difference(normalize_angle(l_wheel_phi_new), m_l_wheel_phi);
		double d_phi_r = normalize_angular_difference(normalize_angle(r_wheel_phi_new), m_r_wheel_phi);
		// Update absolute wheel angles
		m_l_wheel_phi += d_phi_l;
		m_r_wheel_phi += d_phi_r;
		m_l_wheel_phi = normalize_angle(m_l_wheel_phi);
		m_r_wheel_phi = normalize_angle(m_r_wheel_phi);
		// Compute body twist (unit time)
		double d_theta_b = (d_phi_r*(1-r_slip) - d_phi_l*(1-l_slip)) * m_wheel_radius / m_wheel_base;  // Equations 2-8
		double d_x_b = (d_phi_r*(1-r_slip) + d_phi_l*(1-l_slip)) * m_wheel_radius / 2;  // Equation 1 
		double d_y_b = 0.0;
		Vector2D d_v(d_x_b, d_y_b);
		Twist2D d_q_b(d_v, d_theta_b);  
		Vector2D v;  // zero vector
		Transform2D t_a(v, m_q.theta);  // frame aligned with the world but located at the body
		Twist2D d_q = t_a.change_twist_frame(d_q_b);
		// Update pose
		m_q.theta += d_q.getW();
		normalize_angle(m_q.theta);
		m_q.x += d_q.getVx();
		m_q.y += d_q.getVy();
	}
	
	void DiffDrive::translatePose(RobotPose q)
	{
		m_q.x += q.x;
		m_q.y += q.y;
		m_q.theta += q.theta; 
		normalize_angle(m_q.theta);
	}
	
	void DiffDrive::setPose(RobotPose q)
	{
		m_q = q;
	}
	
	Twist2D DiffDrive::getBodyTwist(double l_wheel_phi_new, double r_wheel_phi_new, double dt) const
	{
		// Determine wheel angle change
		double d_phi_l = l_wheel_phi_new - m_l_wheel_phi;
		double d_phi_r = r_wheel_phi_new - m_r_wheel_phi;
		// Compute body twist (unit time)
		double d_theta_b = ((d_phi_r - d_phi_l) * m_wheel_radius / m_wheel_base)/dt;  // Equations 2-8 (separating changes in position and time)
		double d_x_b = ((d_phi_r + d_phi_l) * m_wheel_radius / 2)/dt;  // Equation 1 (separating changes in position and time)
		double d_y_b = 0.0;
		Vector2D d_v(d_x_b, d_y_b);
		Twist2D d_q_b(d_v, d_theta_b); 
		return d_q_b;
	}
	
	WheelVel DiffDrive::twist2WheelVel(const Twist2D & tw) const
	{
		WheelVel wv;
		wv.l_vel = (2*tw.getVx() - tw.getW()*m_wheel_base)/(2.0*m_wheel_radius);  // Equations 13-14
		wv.r_vel = (2*tw.getVx() + tw.getW()*m_wheel_base)/(2.0*m_wheel_radius);  // Equations 9-12
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
