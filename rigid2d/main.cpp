#include<iostream>
#include "rigid2d.hpp"

using namespace rigid2d;

int main(void) 
{
	std::cout << "Enter transform Tab as <dtheta(degrees) dx dy>\n";
	Transform2D t_ab;
	std::cin >> t_ab;
	std::cout << "Enter transform Tbc as <dtheta(degrees) dx dy>\n";
	Transform2D t_bc;
	std::cin >> t_bc;
	std::cout << "Tab is " << t_ab << '\n';
	Transform2D t_ba = t_ab.inv();
	std::cout << "Tba is " << t_ba << '\n';
	std::cout << "Tbc is " << t_bc << '\n';
	Transform2D t_cb = t_bc.inv();
	std::cout << "Tcb is " << t_cb << '\n';
	Transform2D t_ac = t_ab*t_bc;
	std::cout << "Tac is " << t_ac << '\n';
	Transform2D t_ca = t_ac.inv();
	std::cout << "Tca is " << t_ca << '\n';
	
	std::cout << "Enter a vector v as <vx vy> and the frame in which it is defined (a, b or c)\n";
	char frame;
	Vector2D v;
	std::cin >> v >> frame;
	
	if (frame == 'a')
	{
		std::cout << "The vector in frame a is " << v << '\n';
		std::cout << "The vector in frame b is " << t_ba(v) << '\n';
		std::cout << "The vector in frame c is " << t_ca(v) << '\n';
	}
	else if (frame == 'b')
	{
		std::cout << "The vector in frame a is " << t_ab(v) << '\n';
		std::cout << "The vector in frame b is " << v << '\n';
		std::cout << "The vector in frame c is " << t_cb(v) << '\n';
	}
	else if (frame == 'c')
	{
		std::cout << "The vector in frame a is " << t_ac(v) << '\n';
		std::cout << "The vector in frame b is " << t_ac(v) << '\n';
		std::cout << "The vector in frame c is " << v << '\n';
	}
	
	return 0;
}
	
	
	
