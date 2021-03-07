#include <catch_ros/catch.hpp>
#include "nuslam/circle_learning.hpp"
#include "rigid2d/rigid2d.hpp"

TEST_CASE("circle fitting algorithm results", "[circle_fit]")
{
	using namespace rigid2d;
	using namespace nuslam;
	
	Vector2D p1(1,7);
	Vector2D p2(2,6);
	Vector2D p3(5,8);
	Vector2D p4(7,7);
	Vector2D p5(9,5);
	Vector2D p6(3,7);
	std::vector<Vector2D> pts1 = {p1, p2, p3, p4, p5, p6};
	Circle c1 = fit_circle(pts1);
	REQUIRE(almost_equal(c1.x, 4.615482, 1.0e-4));
	REQUIRE(almost_equal(c1.y, 2.807354, 1.0e-4));
	REQUIRE(almost_equal(c1.r, 4.8275, 1.0e-4));
	
	Vector2D p7(-1,0);
	Vector2D p8(-0.3,-0.06);
	Vector2D p9(0.3,0.1);
	Vector2D p10(1,0);
	std::vector<Vector2D> pts2 = {p7, p8, p9, p10};
	Circle c2 = fit_circle(pts2);
	REQUIRE(almost_equal(c2.x, 0.4908357, 1.0e-4));
	REQUIRE(almost_equal(c2.y, -22.15212, 1.0e-4));
	REQUIRE(almost_equal(c2.r, 22.17979, 1.0e-4));
}
