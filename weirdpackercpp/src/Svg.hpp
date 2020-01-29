#ifndef SVG_HPP
#define SVG_HPP

#include<memory>
#include<iostream>
#include "Shape.hpp"

namespace wp
{
class SVG
{
protected:
	class Impl;
	std::shared_ptr<Impl> impl;

public:
	SVG();
	friend std::ostream& operator<<(std::ostream&,const SVG&);
	friend std::istream& operator>>(std::istream&,SVG&);

	std::vector<wp::Shape> getAllShapes() const;
};

std::ostream& operator<<(std::ostream&,const SVG&);
std::istream& operator>>(std::istream&,SVG&);

}




#endif
