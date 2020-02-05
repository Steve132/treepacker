#ifndef SHAPE_HPP
#define SHAPE_HPP

#include<Eigen/Dense>
#include<vector>
#include<array>
#include<iostream>

namespace wp
{

typedef Eigen::Vector2f Point;
typedef std::vector<Point> PolyLine;

class Shape
{
public:
	std::string id;
	PolyLine outerline;
	std::vector<PolyLine> holes;

	friend std::ostream& operator<<(std::ostream&,const Shape&);
	friend std::istream& operator>>(std::istream&,Shape&);
};
std::ostream& operator<<(std::ostream&,const Shape&);
std::istream& operator>>(std::istream&,Shape&);
}

#endif
