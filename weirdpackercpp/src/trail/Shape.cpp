#include "Shape.hpp"
#include <algorithm>
#include <iterator>
using namespace trail;


static std::vector<Point> readPoints(std::istream& inp,size_t N)
{
	std::vector<Point> out;
	for(size_t i=0;i<N;i++)
	{
		float x,y;
		inp >> x >> y;
		out.emplace_back(x,y);
	}
	return out;
}
static void writePoints(std::ostream& out,const std::vector<Point>& p)
{
	out << p.size();
	out << " ";
	for(size_t i=0;i<p.size();i++)
	{
		out << p[i][0] << " " << p[i][1] << " ";
	}
}
std::istream& trail::operator>>(std::istream& inp,Shape& shape)
{
	inp >> shape.id;
	size_t num_verts;
	inp >> num_verts;
	
	shape.outerline=readPoints(inp,num_verts);
	size_t num_holes;
	inp >> num_holes;
	for(size_t i=0;i<num_holes;i++)
	{
		inp >> num_verts;
		shape.holes.push_back(readPoints(inp,num_verts));
	}
	return inp;
}
std::ostream& trail::operator<<(std::ostream& out,const Shape& shape)
{
	out << shape.id << " ";
	writePoints(out,shape.outerline);
	out << shape.holes.size() << " ";
	for(size_t i=0;i<shape.holes.size();i++)
	{
		writePoints(out,shape.holes[i]);
	}
	return out;
}
