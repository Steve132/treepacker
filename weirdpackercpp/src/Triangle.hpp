#ifndef TRIANGLE_HPP
#define TRIANGLE_HPP

#include "Shape.hpp"
#include<Eigen/Geometry>

namespace wp
{

class Triangle
{
public:
	std::array<trail::Point,3> p;
	float area2() const;
	float area() const;
	Eigen::AlignedBox2f bounds() const;
	
	static bool intersect(const Triangle& a,const Triangle& b);
	static Triangle transform(const Eigen::Matrix<float,2,3>& Rt,const Triangle& tri);
	Eigen::Vector3f smallest_enclosing_circle() const;
	Eigen::Vector3f mean_enclosing_circle() const;
};


}

#include "Triangle.inl"

#endif
