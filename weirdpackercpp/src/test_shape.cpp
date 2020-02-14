#include "Shape.hpp"
#include "Mesh.hpp"
#include<iostream>
#include "BallTree.hpp"
#include<fstream>
#include<thread>
#include "Svg.hpp"
#include "Renderer.hpp"


typedef balltree<wp::Triangle,2,float> balltree2f;

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

typename balltree2f::ball aabb_parentfunc(
	const typename balltree2f::ball& lref,
	const typename balltree2f::ball& rref,
	const typename balltree2f::ball* bb,
	const typename balltree2f::ball* be
)
{
	
	Eigen::AlignedBox2f bbox=bb[0].leaf.bounds();
	size_t n=be-bb;
	//std::cerr << "The N: " << n << std::endl;
	//std::cerr << bbox.min().transpose() << ":" << bbox.max().transpose() << std::endl;
	for(size_t i=1;i<n;i++)
	{
		bbox.extend(bb[i].leaf.bounds());
	//	std::cerr << bbox.min().transpose() << ":" << bbox.max().transpose() << std::endl;
	}
	if(n==14)
	{
		std::cerr << "Start debugging here";
	}
	typename balltree2f::ball parent(lref,rref);
	//std::cerr << "before:" << parent.position.transpose() << "," << parent.radius << std::endl;
	
	double newrad=(bbox.max()-bbox.min()).norm()/2.0f;
	if(newrad < parent.radius)
	{
		parent.position=(bbox.max()+bbox.min())/2.0f;
		parent.radius=newrad;
	}
	//std::cerr << "after: " << parent.position.transpose() << "," << parent.radius << std::endl;
	//std::cerr << lref.num_children << std::endl;	
	return parent;
}
balltree2f build_btree(const std::vector<wp::Triangle>& tris)
{
	std::vector<balltree2f::ball> all_balls;
	for(size_t i=0;i<tris.size();i++)
	{
		Eigen::Vector3f ecirc=tris[i].smallest_enclosing_circle();
		all_balls.emplace_back(tris[i],Eigen::Vector2f(ecirc.x(),ecirc.y()),ecirc.z());
	}
	
	balltree2f bt(all_balls.data(),all_balls.data()+all_balls.size(),aabb_parentfunc);
	return bt;
}


typename balltree2f::ball getTestTBall()
{

	typename balltree2f::ball test;
	
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
	balltree2f btree=build_btree(msh.triangles);

	std::vector<wp::Triangle> tris2=wp::Mesh::triangulate(shapes[1]);
	wp::Mesh msh2(tris2);
	msh2.transform_in_place(Rt);
	std::cout << "num_triangles: " << msh2.triangles.size() << std::endl;
	balltree2f btree2=build_btree(msh2.triangles);

	//typename balltree2f::ball testball=getTestTBall();
	

	wp::Renderer r({800,600},3.0f);
	r.clear({0x00,0x00,0x40});	
	
	std::cout << "all nodes num children:" << std::endl;
	for(int i = 0; i < btree.allnodes.size(); i++)
	{
		std::cout << btree.allnodes[i].num_children << ' ';
	}
	std::cout << std::endl;

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
		Rt2.col(2)=r.getMousePosition()-Eigen::Vector2f(1.0,1.0);
		
		//std::cout << Rt2 << std::endl;
		wp::Mesh msh2t=msh2;
		msh2t.transform_in_place(Rt2);
		balltree2f btree2t=btree2;
		balltransform<2,float> btform(Rt2.col(2));
		
		for(size_t i=0;i<btree2t.allnodes.size();i++)
		{
			btree2t.allnodes[i].transform_in_place(btform);
			//TODO: btree2t.allnodes[i].leaf.transform_in_place(Rt2);
			btree2t.allnodes[i].leaf = wp::Triangle::transform(Rt2, btree2t.allnodes[i].leaf);
		}
		
		r.draw(btree2t.allnodes[1],{0x00,0x40,0x00},true);
		
		bool inter=false;
		//inter=btree.intersect(btree2t.allnodes[0],wp::Triangle::intersect);
		//inter=btree.intersect(btree2t.allnodes[1],wp::Triangle::intersect);
		inter=btree.intersect_callable([&btree2t](const typename balltree2f::ball& thisnode1)
		{
			return btree2t.intersect_callable([&thisnode1](const typename balltree2f::ball& thisnode2)
			{
				return thisnode1.intersect(thisnode2, wp::Triangle::intersect);
			});
		});
		
		
		
		uint8_t cv=inter ? 0xFF : 0x00;
		r.draw(msh2t,{cv,~cv,0x00},true);
		r.update();
	}
	
	return 0;
}

