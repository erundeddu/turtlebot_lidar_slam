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
		, m_wheel_base(wheel_base)
		, m_wheel_radius(wheel_radius)
		, m_r_wheel_phi(0.0)
		, m_l_wheel_phi(0.0)
	{
	}
	
	void DiffDrive::UpdatePose(r_wheel_phi_new, l_wheel_phi_new)
	{
		// Determine wheel angle change
		double d_phi_r = r_wheel_phi_new - m_r_wheel_phi;
		double d_phi_l = l_wheel_phi_new - m_l_wheel_phi;
		// Update absolute wheel angles
		m_r_wheel_phi += d_phi_r;
		m_l_wheel_phi += d_phi_l;
		// Compute body twist
		double d_theta_b = (d_phi_r - d_phi_l) * wheel_radius / wheel_base;
		double d_x_b = (d_phi_r + d_phi_l) * wheel_radius / 2;
		double d_y_b = 0.0;
		Vector2D d_v(d_x_b, d_y_b);
		Twist2D d_q_b(d_v, d_theta_b);  
		Vector2D v;  // zero vector
		Transform2D t_a(v, m_theta);  // frame aligned with the world but located at the body
		Twist2D d_q = t_a.change_twist_frame(d_q_b);
		// Update pose
		m_theta += d_q.getW();
		m_x += d_q.getX();
		m_y += d_q.getY();
	}
	
}
