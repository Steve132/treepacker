#ifndef SHAPE_HPP
#define SHAPE_HPP

#include<Eigen/Dense>
#include<vector>
#include<array>

namespace wp
{

typedef Eigen::Vector2f Point;
typedef std::array<Point,3> Triangle;
typedef std::vector<Point> PolyLine;

class Shape
{
public:
	std::string id;
	PolyLine outerline;
	std::vector<PolyLine> holes;
};

}

#endif
