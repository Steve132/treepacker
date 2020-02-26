#include "Shape.hpp"
#include "Mesh.hpp"
#include<iostream>
#include "BallTree.hpp"
#include<fstream>
#include<thread>
#include "Svg.hpp"
#include "Renderer.hpp"
#include "Cutout.hpp"
#include <chrono>
#include "SampleWithoutReplacement.hpp"
#include <vector>
#include <unordered_map>
#include <gperftools/profiler.h>
#include "CutoutSet.hpp"


int main(int argc,char** argv)
{
	randomWeightTest();
	//splitDBtest();

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
	
	int selected=0;
	wp::CutoutSet cs;
	cs.add(cutouts[0],wp::CutoutSet::generate_orientations(90,false));
	cs.add(cutouts[1],wp::CutoutSet::generate_orientations(90,false));
	
	wp::balltransform2f mouse_tform;
	auto start=std::chrono::high_resolution_clock::now();
	static const size_t N=1000;
	//#pragma omp parallel for
	for(size_t i=0;i<N;i++)
	{
		mouse_tform=find_first_fit(cutouts[0],cs.cutouts[1][selected],Eigen::Vector2f(0.01,0.01),Eigen::Vector2f(5.0f,3.0f));
	}
	auto dur=std::chrono::high_resolution_clock::now()-start;
	std::cerr << static_cast<double>(N)/std::chrono::duration<double>(dur).count() << std::endl;

	std::cerr << "mouse" << mouse_tform.translation << std::endl;
	wp::Renderer r({800,600},3.0f);
	r.clear({0x00,0x00,0x40});	
	
	
	
	while(r.isOpen())
	{
		r.clear({0x00,0x00,0x40});
		//r.draw(msh,{0x40,0x40,0x00});
		r.draw(cutouts[0].mesh,{0xFF,0xFF,0x00},true);
		for(size_t i=0;i<cutouts[0].tree.allnodes.size();i++)
		{
			r.draw(cutouts[0].tree.allnodes[i],{0x40,0x00,0x00},true);
		}
		
		//mouse_tform.translation=r.getMousePosition();
		if(r.key_released("SPACE"))
		{
			std::cout << "Spacebar" << std::endl;
			selected=(selected+1) % cs.cutouts[1].size();
			//ProfilerStart("nameOfProfile.log");
			
			//ProfilerStop();
		}
		//std::cout << Rt2 << std::endl;
		wp::Cutout temp_cut2=cs.cutouts[1][selected];
		temp_cut2.transform_in_place(mouse_tform);
		
		bool inter=cutouts[0].intersect(cs.cutouts[1][selected],mouse_tform);
		
		uint8_t cv=inter ? 0xFF : 0x00;
		r.draw(temp_cut2.mesh,{cv,~cv,0x00},true);
		r.update();
	}
	
	return 0;
}

