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

typename balltree<wp::Triangle,2,float>::ball aabb_parentfunc(
	const typename balltree<wp::Triangle,2,float>::ball& lref,
	const typename balltree<wp::Triangle,2,float>::ball& rref,
	const typename balltree<wp::Triangle,2,float>::ball* bb,
	const typename balltree<wp::Triangle,2,float>::ball* be
)
{
	Eigen::AlignedBox2f bbox=bb[0].leaf.bounds();
	size_t n=be-bb;
	for(size_t i=1;i<n;i++)
	{
		bbox.extend(bb[i].leaf.bounds());
	}
	
	typename balltree<wp::Triangle,2,float>::ball parent(lref,rref);
	parent.position=(bbox.max()+bbox.min())/2.0f;
	parent.radius=(bbox.max()-bbox.min()).norm()/2.0f;
	return parent;
}
balltree<wp::Triangle,2,float> build_btree(const std::vector<wp::Triangle>& tris)
{
	std::vector<balltree<wp::Triangle,2,float>::ball> all_balls;
	for(size_t i=0;i<tris.size();i++)
	{
		Eigen::Vector3f ecirc=tris[i].smallest_enclosing_circle();
		all_balls.emplace_back(tris[i],Eigen::Vector2f(ecirc.x(),ecirc.y()),ecirc.z());
	}
	
	balltree<wp::Triangle,2,float> bt(all_balls.data(),all_balls.data()+all_balls.size(),aabb_parentfunc);
	return bt;
}


typename balltree<wp::Triangle,2,float>::ball getTestTBall()
{

	typename balltree<wp::Triangle,2,float>::ball test;
	
	test.radius=0.1;
	test.leaf=wp::Triangle{
		trail::Point(0.0,1.0)*test.radius,
		trail::Point(0.86602540378,-0.5)*test.radius,
		trail::Point(-0.86602540378,-0.5)*test.radius
	};
	Eigen::Vector3f ecirc=test.leaf.smallest_enclosing_circle();
	test.position=Eigen::Vector2f(ecirc.x(),ecirc.y());
	test.radius=ecirc.z();
	return test;
}


int main(int argc,char** argv)
{
	std::ifstream inpf("../../../data/drawing2.svg");
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
	
	std::vector<wp::Triangle> tris2=wp::Mesh::triangulate(shapes[1]);
	wp::Mesh msh2(tris2);
	msh2.transform_in_place(Rt);
	balltree<wp::Triangle,2,float> btree2=build_btree(msh2.triangles);

	//typename balltree<wp::Triangle,2,float>::ball testball=getTestTBall();
	

	wp::Renderer r({800,600},3.0f);
	r.clear({0x00,0x00,0x40});	
	while(r.isOpen())
	{
		r.clear({0x00,0x00,0x40});
		//r.draw(msh,{0x40,0x40,0x00});
		r.draw(msh,{0xFF,0xFF,0x00},true);
		for(size_t i=0;i<btree.allnodes.size();i++)
		{
			r.draw(btree.allnodes[i],{0x40,0x00,0x00},true);
		}
		
		Eigen::Matrix<float,2,3> Rt2=Eigen::Matrix<float,2,3>::Identity();
		Rt2.col(2)=r.getMousePosition();
		
		std::cout << Rt2 << std::endl;
		r.draw(msh2,{0x00,0x00,0xFF},true);
		
		//bool inter=btree.intersect(tb2,wp::Triangle::intersect);
		//uint8_t cv=inter ? 0xFF : 0x00;
		//r.draw(tb2.leaf,{cv,~cv,0x00},true);
		r.update();
	}
	
	return 0;
}

