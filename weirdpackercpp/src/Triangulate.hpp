#ifndef TRIANGULATE_HPP
#define TRIANGULATE_HPP

#include "Shape.hpp"

namespace wp
{

typedef std::array<Point,3> Triangle;
std::vector<Triangle> triangulate(const Shape& shape);

}

#endif
