#ifndef TRIANGULATE_HPP
#define TRIANGULATE_HPP

#include "Shape.hpp"

namespace wp
{

typedef std::array<Point,3> Triangle;
std::vector<Triangle> triangulate(const Shape& shape);

namespace {

inline bool crossIntersect(const Triangle& t,const Point& b,const Point& c,float normal)
{
	Point cbr(c.y()-b.y(),-(c.x()-b.y()));
	cbr*=normal;
	float bcbr=b.dot(cbr);
	return ((t[0].dot(cbr)-bcbr) < 0) && ((t[1].dot(cbr)-bcbr) < 0) && ((t[2].dot(cbr)-bcbr) < 0);
}
inline float computeNormal(const Triangle& t)
{
	Point ba=t[1]-t[0];
	Point ca=t[2]-t[0];
	ca=Point(ca.y(),-ca.x());
	return ba.dot(ca);
}
// test if one of the triangles has a side with all of the other triangles points on the outside.  Assume that we know all the normals of ech triangle, which tells us the direction that the points are specified.
inline bool triangleIntersect3(const Triangle& t0,const Triangle& t1)
{
	float normal0 = computeNormal(t0);
	float normal1 = computeNormal(t1);
	
	return 
	crossIntersect(t1, t0[0], t0[1], normal0) &&
	crossIntersect(t1, t0[1], t0[2], normal0) &&
	crossIntersect(t1, t0[2], t0[0], normal0) &&
	crossIntersect(t0, t1[0], t1[1], normal1) &&
	crossIntersect(t0, t1[1], t1[2], normal1) &&
	crossIntersect(t0, t1[2], t1[0], normal1);
}
}
inline bool intersect(const Triangle& t1,const Triangle& t2)
{
		return triangleIntersect3(t1,t2);
}




}

#endif
