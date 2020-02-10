#include "Triangle.hpp"
#include<algorithm>

//https://en.wikipedia.org/wiki/Smallest-circle_problem
Eigen::Vector3f wp::Triangle::smallest_enclosing_circle() const
{
	Eigen::Vector2f v[3]={
		(p[0]-p[1]),
		(p[1]-p[2]),
		(p[2]-p[0])
	};
	float m[3]={
		v[0].squaredNorm(),
		v[1].squaredNorm(),
		v[2].squaredNorm()
	};
	bool isacute=(m[0]+m[1] > m[2]) && (m[1]+m[2] > m[0]) && (m[2]+m[0] > m[1]);
	Eigen::Vector2f cp;
	float rad;
	
	if(isacute)
	{
		//do circumscribed circle
		Eigen::Vector2f lp=
				v[1]*p[0].squaredNorm()
				+v[2]*p[1].squaredNorm()
				+v[0]*p[2].squaredNorm();
		cp=Eigen::Vector2f(lp.y(),-lp.x());
		cp/=2.0f*(v[1].y()*p[0].x()+v[2].y()*p[1].x()+v[0].y()*p[2].x());
		rad=(p[0]-cp).norm();
	}
	else
	{
		size_t maxdex=std::max_element(m,m+3)-m;
		Eigen::Vector2f sv1=p[maxdex],sv0=p[(maxdex+1)%3];
		cp=(sv1+sv0)/2.0f;
		rad=(sv1-sv0).norm()/2.0f;
	}
	return Eigen::Vector3f(cp.x(),cp.y(),rad);
	
}

Eigen::Vector3f wp::Triangle::mean_enclosing_circle() const
{
	trail::Point position=(p[0]+p[1]+p[2])/3.0;
	std::array<double,3> distances={
		(p[0]-position).norm(),
		(p[1]-position).norm(),
		(p[2]-position).norm()
	};
	double rad=*std::max_element(&distances[0],&distances[0]+3);
	return Eigen::Vector3f(position.x(),position.y(),rad);
}
