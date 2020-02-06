#ifndef TRIANGULATE_HPP
#define TRIANGULATE_HPP

#include "Shape.hpp"

namespace wp
{

typedef std::array<trail::Point,3> Triangle;
std::vector<Triangle> triangulate(const trail::Shape& shape);

class TriangulatedShape
{
public:
	std::vector<Triangle> triangles;
	TriangulatedShape(const trail::Shape& sh);
	float outer_area;
	std::vector<float> holes_area;
	
	
};
namespace {

inline bool crossIntersect(const Triangle& points,const Triangle& tri)
{
	Eigen::Matrix<float,3,2> dABC;
	dABC << (points[0]-tri[2]).transpose(),
			(points[1]-tri[2]).transpose(),
			(points[2]-tri[2]).transpose();

	Eigen::Vector2f d12=tri[1]-tri[2];
	Eigen::Vector2f d20=tri[2]-tri[0];
      
    float D = d12.y() * d20.x() - d12.x() * d20.y();
    D=-D;

    Eigen::Vector3f s = dABC.leftCols<1>()*d12.y() - dABC.rightCols<1>()*d12.x();
    Eigen::Vector3f t = dABC.leftCols<1>()*d20.y() - dABC.rightCols<1>()*d20.x();
    
    if(D < 0.0)
    {
        s=-s;
        t=-t;
        D=-D;
    }

    return (s.array() <= 0.0f).all()
        || (t.array() <= 0.0f).all()
        || ((s+t).array() >= D).all();
}
// test if one of the triangles has a side with all of the other triangles points on the outside.  Assume that we know all the normals of ech triangle, which tells us the direction that the points are specified.
inline bool triangleIntersect(const Triangle& t0,const Triangle& t1)
{
	!(crossIntersect(t0,t1) || crossIntersect(t1,t0));
}
}
inline bool triangle_intersect(const Triangle& t1,const Triangle& t2)
{
		return triangleIntersect(t1,t2);
}




}

#endif
