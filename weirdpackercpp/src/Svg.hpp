#ifndef SVG_HPP
#define SVG_HPP

#include<memory>
#include<iostream>




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
	
};

std::ostream& operator<<(std::ostream&,const SVG&);

}




#endif
