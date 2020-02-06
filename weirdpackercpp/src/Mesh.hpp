#ifndef MESH_HPP
#define MESH_HPP

#include "Triangle.hpp"

namespace wp
{
class Mesh
{
public:
	std::vector<Triangle> triangles;
	
	Mesh(const std::vector<Triangle>& tris);
	//Todo maybe use holes?
	float area;
	//float outer_area;
	//std::vector<float> holes_area;
	Eigen::AlignedBox2f bounding_box;
	
	static Mesh transform(const Eigen::Matrix<float,2,3>& Rt,const Mesh& trishape);
	static std::vector<Triangle> triangulate(const trail::Shape& shape);
};
}

#endif
