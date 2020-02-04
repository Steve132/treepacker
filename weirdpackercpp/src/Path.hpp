#ifndef PATH_HPP
#define PATH_HPP

#include "Shape.hpp"
#include<iostream>

namespace wp
{
	wp::Shape path2shape(std::istream& inp,double points_per_unit);
}

#endif
