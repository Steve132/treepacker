#include "Shape.hpp"
#include "Mesh.hpp"
#include<iostream>
#include "BallTree.hpp"
#include<fstream>
#include<thread>
#include "Svg.hpp"
#include "Renderer.hpp"
#include "Cutout.hpp"

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

	std::ifstream inpf("../../../data/drawing2tri.svg");
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
	std::vector<wp::Triangle> tris2=wp::Mesh::triangulate(shapes[1]);
	
	wp::Mesh msh(tris);
	wp::Mesh msh2(tris2);
	
	wp::balltransform2f rtform;
	rtform.scale=(1.0f/msh.bounding_box.sizes().maxCoeff());
	rtform.translation=-msh.bounding_box.min()*rtform.scale;
	
	wp::Cutout cut1(msh);
	cut1.transform_in_place(rtform);
	
	wp::Cutout cut2(msh2);
	cut2.transform_in_place(rtform);
	
	
	std::cout << "num_triangles: " << msh2.triangles.size() << std::endl;

	wp::Renderer r({800,600},3.0f);
	r.clear({0x00,0x00,0x40});	
	
	std::cout << "Num Leaves: " << msh2.triangles.size() << std::endl;
	std::cout << "Num Leaves: " << msh.triangles.size() << std::endl;
	
	wp::balltransform2f mouse_tform;
	
	while(r.isOpen())
	{
		r.clear({0x00,0x00,0x40});
		//r.draw(msh,{0x40,0x40,0x00});
		r.draw(cut1.mesh,{0xFF,0xFF,0x00},true);
		for(size_t i=0;i<cut1.tree.allnodes.size();i++)
		{
			r.draw(cut1.tree.allnodes[i],{0x40,0x00,0x00},true);
		}
		
		mouse_tform.translation=r.getMousePosition()-Eigen::Vector2f(1.0,1.0);
		
		//std::cout << Rt2 << std::endl;
		wp::Cutout temp_cut2=cut2;
		temp_cut2.transform_in_place(mouse_tform);
		
		bool inter=cut1.intersect(temp_cut2);
		
		uint8_t cv=inter ? 0xFF : 0x00;
		r.draw(temp_cut2.mesh,{cv,~cv,0x00},true);
		r.update();
	}
	
	return 0;
}

