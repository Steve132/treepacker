#include "Shape.hpp"
#include "Mesh.hpp"
#include<iostream>
#include "BallTree.hpp"
#include<fstream>
#include<thread>
#include "Svg.hpp"
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
balltree<wp::Triangle,2,float> build_btree(const std::vector<wp::Triangle>& tris)
{
	std::vector<balltree<wp::Triangle,2,float>::ball> all_balls;
	for(size_t i=0;i<tris.size();i++)
	{
		Eigen::Vector3f ecirc=tris[i].smallest_enclosing_circle();
		all_balls.emplace_back(tris[i],Eigen::Vector2f(ecirc.x(),ecirc.y()),ecirc.z());
	}
	
	balltree<wp::Triangle,2,float> bt(all_balls.data(),all_balls.data()+all_balls.size());
	return bt;
}

int main(int argc,char** argv)
{
	std::ifstream inpf("../../../data/drawing.svg");
	trail::SVG drawing;
	inpf >> drawing;
	std::vector<trail::Shape> shapes=drawing.getAllShapes();
	
	for(size_t i=0;i<shapes.size();i++)
	{
		std::cout << "shape[" << i << "]:" << std::endl;
		shapes[i].cleanup();
		std::cout << shapes[i] << std::endl;
	}
	std::vector<wp::Triangle> tris=wp::Mesh::triangulate(shapes[0]);

	wp::Mesh msh(tris);
	Eigen::Matrix<float,2,3> Rt;
	Rt << Eigen::Matrix2f::Identity(),-msh.bounding_box.min();
	Rt *= (1.0f/msh.bounding_box.sizes().maxCoeff());
	Rt.col(2).array()+=1.0;
	std::cout << Rt << std::endl;
	msh.transform_in_place(Rt); //TODO: using this makes the intersection checking O(n)!  MUST add transform to intersect!
	balltree<wp::Triangle,2,float> btree=build_btree(msh.triangles);
	wp::Renderer r({800,600},3.0f);
	r.clear({0x00,0x00,0x40});
	while(r.isOpen())
	{
		//r.draw(msh,{0x40,0x40,0x00});
		r.draw(msh,{0xFF,0xFF,0x00},true);
		
		for(size_t i=0;i<btree.allnodes.size();i++)
		{
			if(btree.allnodes[i].num_children<=2)
			{
				r.draw(btree.allnodes[i],{0xFF,0x00,0x00},true);
			}
		}
		r.update();
	}
	
	return 0;
}

int oldmain(int argc,char** argv)
{
	trail::Shape input;

	std::ifstream inf("../../../data/square.shape");
	inf >> input;
	
	std::vector<wp::Triangle> tris=wp::Mesh::triangulate(input);
	
	printTriangles(tris);
	
	
	balltree<wp::Triangle,2,float> balltree=build_btree(tris);
	
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
	
	
	//std::cout << "Intersected: " << balltree.intersect(test,wp::Triangle::intersect);
	
	return 0;
}
