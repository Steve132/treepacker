#ifndef PATH_HPP
#define PATH_HPP

#include "Shape.hpp"
#include<iostream>

namespace trail
{
	trail::Shape path2shape(std::istream& inp,const std::string& name,double points_per_unit);
}

#endif
