#include "Shape.hpp"
#include "Mesh.hpp"
#include<iostream>
#include "BallTree.hpp"
#include<fstream>
#include<thread>
#include "Svg.hpp"
#include "Renderer.hpp"
#include "Cutout.hpp"

int main(int argc,char** argv)
{
	std::ifstream inpf("../../../data/drawing2tri.svg");
	trail::SVG drawing;
	inpf >> drawing;
	std::vector<trail::Shape> shapes=drawing.getAllShapes();
	std::vector<wp::Cutout> cutouts;
	
	wp::balltransform2f rtform;
	
	rtform.scale=(1.0f/136.0);
	for(size_t i=0;i<shapes.size();i++)
	{
		std::cout << "shape[" << i << "]:" << std::endl;
		shapes[i].cleanup();
		std::cout << shapes[i] << std::endl;
		std::vector<wp::Triangle> tris=wp::Mesh::triangulate(shapes[i]);
		wp::Mesh msh(tris);
		cutouts.emplace_back(msh);
		wp::balltransform2f rtform2=rtform;
		rtform2.translation=-cutouts.back().mesh.bounding_box.min()*rtform.scale;
		cutouts.back().transform_in_place(rtform2);
	}
	
	wp::Renderer r({800,600},3.0f);
	r.clear({0x00,0x00,0x40});	
	
	wp::balltransform2f mouse_tform;
	
	while(r.isOpen())
	{
		r.clear({0x00,0x00,0x40});
		//r.draw(msh,{0x40,0x40,0x00});
		r.draw(cutouts[0].mesh,{0xFF,0xFF,0x00},true);
		for(size_t i=0;i<cutouts[0].tree.allnodes.size();i++)
		{
			r.draw(cutouts[0].tree.allnodes[i],{0x40,0x00,0x00},true);
		}
		
		mouse_tform.translation=r.getMousePosition();
		if(r.key_released("SPACE"))
		{
			std::cout << "Spacebar" << std::endl;
			cutouts[0].merge_in_place(cutouts[1],mouse_tform);
		}
		//std::cout << Rt2 << std::endl;
		wp::Cutout temp_cut2=cutouts[1];
		temp_cut2.transform_in_place(mouse_tform);
		
		bool inter=cutouts[0].intersect(cutouts[1],mouse_tform);
		
		uint8_t cv=inter ? 0xFF : 0x00;
		r.draw(temp_cut2.mesh,{cv,~cv,0x00},true);
		r.update();
	}
	
	return 0;
}


