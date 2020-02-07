#include "Shape.hpp"
#include "Mesh.hpp"
#include<iostream>
#include "BallTree.hpp"
#include<fstream>
#include<thread>
#include "Renderer.hpp"

void printTriangles(const std::vector<wp::Triangle>& tris)
{
	for(size_t i=0;i<tris.size();i++)
	{
		for(size_t d=0;d<3;d++)
		{
			std::cout << "(" << tris[i].p[d][0] << ","  << tris[i].p[d][1] << ")";
		}
		std::cout << std::endl;
	}	
}

int main(int argc,char** argv)
{
	trail::Shape input;

	std::ifstream inf("../../../data/square.shape");
	inf >> input;
	
	std::vector<wp::Triangle> tris=wp::Mesh::triangulate(input);
	
	printTriangles(tris);
	
	std::vector<balltree<wp::Triangle,2,float>::ball> all_balls;
	for(size_t i=0;i<tris.size();i++)
	{
		trail::Point position=(tris[i].p[0]+tris[i].p[1]+tris[i].p[2])/3.0;
		std::array<double,3> distances={
			(tris[i].p[0]-position).norm(),
			(tris[i].p[1]-position).norm(),
			(tris[i].p[2]-position).norm()
		};
		double rad=*std::max_element(&distances[0],&distances[0]+3);
		all_balls.emplace_back(tris[i],position,rad);
	}
	
	balltree<wp::Triangle,2,float> balltree(all_balls.data(),all_balls.data()+all_balls.size());
	
	for(size_t i=0;i<balltree.allnodes.size();i++)
	{
		const auto& thisball=balltree.allnodes[i];
		std::cout << i << ":" << thisball.position.transpose() << " : " << thisball.radius << std::endl;
	}
	
	typename balltree<wp::Triangle,2,float>::ball test;
	
	test.position=trail::Point(1.15,.5);
	test.radius=0.1;
	test.leaf=wp::Triangle{
		test.position+trail::Point(0.0,1.0)*test.radius,
		test.position+trail::Point(0.86602540378,-0.5)*test.radius,
		test.position+trail::Point(-0.86602540378,-0.5)*test.radius
	};
	
	wp::Mesh msh(tris);
	Eigen::Matrix<float,2,3> Rt;
	Rt << Eigen::Matrix2f::Identity(),-msh.bounding_box.min();
	Rt *= (1.0f/msh.bounding_box.sizes().maxCoeff());
	std::cout << Rt << std::endl;
	msh.transform_in_place(Rt);
	
	printTriangles(msh.triangles);
	
	wp::Renderer r({800,600},6.0f);
	r.clear({0x00,0x00,0x40});
	while(r.isOpen())
	{
		for(size_t i=0;i<msh.triangles.size();i++)
		{
			r.draw(msh.triangles[i],{0xFF,0xFF,0x00});
		}
		r.update();
	}
	//std::cout << "Intersected: " << balltree.intersect(test,wp::Triangle::intersect);
	
	return 0;
}
