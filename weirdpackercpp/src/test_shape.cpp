#include "Shape.hpp"
#include "Triangulate.hpp"
#include<iostream>
#include "BallTree.hpp"
#include<fstream>
#include<thread>


int main(int argc,char** argv)
{
	wp::Shape input;

	std::ifstream inf("../../../data/square.shape");
	inf >> input;
	
	std::vector<wp::Triangle> tris=wp::triangulate(input);
	
	for(size_t i=0;i<tris.size();i++)
	{
		for(size_t d=0;d<3;d++)
		{
			std::cout << "(" << tris[i][d][0] << ","  << tris[i][d][1] << ")";
		}
		std::cout << std::endl;
	}
	
	std::vector<balltree<wp::Triangle,2,float>::ball> all_balls;
	for(size_t i=0;i<tris.size();i++)
	{
		wp::Point position=(tris[i][0]+tris[i][1]+tris[i][2])/3.0;
		std::array<double,3> distances={
			(tris[i][0]-position).norm(),
			(tris[i][1]-position).norm(),
			(tris[i][2]-position).norm()
		};
		double rad=*std::max_element(&distances[0],&distances[0]+3);
		all_balls.emplace_back(tris[i],rad,position);
	}
	
	balltree<wp::Triangle,2,float> balltree(all_balls.data(),all_balls.data()+all_balls.size());
	
	for(size_t i=0;i<balltree.allnodes.size();i++)
	{
		std::cout << i << ":" << balltree.allnodes[i].boundary.position.transpose() << " : " << balltree.allnodes[i].boundary.radius << std::endl;
		std::cout << ":" << balltree.allnodes[i].leftdex << "," << balltree.allnodes[i].rightdex << std::endl;
	}
	
	typename balltree<wp::Triangle,2,float>::ball test;
	
	test.position=wp::Point(1.15,.5);
	test.radius=0.1;
	test.leaf=wp::Triangle{
		test.position+wp::Point(0.0,1.0)*test.radius,
		test.position+wp::Point(0.86602540378,-0.5)*test.radius,
		test.position+wp::Point(-0.86602540378,-0.5)*test.radius
	};
	
	std::cout << "Intersected: " << balltree.intersect(test,wp::intersect);
	
	return 0;
}
