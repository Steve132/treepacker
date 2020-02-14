#include<cmath>

namespace wp
{
namespace {

inline bool crossIntersect(const Triangle& points,const Triangle& tri)
{
	Eigen::Matrix<float,3,2> dABC;
	dABC << (points.p[0]-tri.p[2]).transpose(),
			(points.p[1]-tri.p[2]).transpose(),
			(points.p[2]-tri.p[2]).transpose();

	Eigen::Vector2f d12=tri.p[1]-tri.p[2];
	Eigen::Vector2f d20=tri.p[2]-tri.p[0];
      
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

}

// test if one of the triangles has a side with all of the other triangles points on the outside.  Assume that we know all the normals of ech triangle, which tells us the direction that the points are specified.
inline bool Triangle::intersect(const Triangle& t0,const Triangle& t1)
{
	return !(crossIntersect(t0,t1) || crossIntersect(t1,t0));
}
inline float Triangle::area2() const
{
	float a2=(p[1]-p[0]).squaredNorm();
	float b2=(p[2]-p[1]).squaredNorm();
	float c2=(p[3]-p[2]).squaredNorm();
	float r1=4.0f*a2*c2;
	float r2=(a2+c2-b2);
	r2*=r2;
	return (r1+r2)/16.0f;
}
inline float Triangle::area() const
{
	return std::sqrt(area2());
}
inline Eigen::AlignedBox2f Triangle::bounds() const
{
	auto lower=p[0].array().min(p[1].array()).min(p[2].array());
	auto upper=p[0].array().max(p[1].array()).max(p[2].array());
	return Eigen::AlignedBox2f(lower,upper);
	//Eigen::AlignedBox2f bb(p[0],p[1]);
	
	//return bb;
}
inline void Triangle::transform_in_place(const Eigen::Matrix<float,2,3>& Rt)
{
	auto R=Rt.leftCols<2>();
	auto t=Rt.rightCols<1>();
	p[0]=R*p[0]+t;
	p[1]=R*p[1]+t;
	p[2]=R*p[2]+t;
}

inline Triangle Triangle::transform(const Eigen::Matrix<float,2,3>& Rt,const Triangle& tri)
{
	Triangle ntri=tri;
	ntri.transform_in_place(Rt);
	return ntri;
}

}
