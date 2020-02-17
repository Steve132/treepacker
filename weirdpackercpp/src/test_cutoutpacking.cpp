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

	
std::vector<size_t> split_dominant_bisect(size_t N)
{
	std::vector<size_t> order(N,N);
	size_t position=0;
	for(size_t increment=N/2;increment > 0;increment/=2)
	{
		for(size_t k=0;k<N;k+=increment)
		{
			if(order[k]==N)
			{
				order[k]=position++;
			}
		}
	}
	std::vector<size_t> output(N);
	for(size_t k=0;k<N;k++)
	{
		size_t v=order[k];
		if(v != N)
		{
			output[v]=k;
		}
	}
	return output;
}

void randomWeightTest()
{
	const double weights[]={10.0,10.0,0.01,0.01};
	std::vector<size_t> outindices(5);
	size_t cnt=3;
	std::default_random_engine rengine(std::chrono::system_clock::now().time_since_epoch().count());
	
	ordered_sample_without_replacement_indices(weights,weights+5,cnt,rengine,outindices.begin());
	for(size_t i=0;i<cnt;i++)
	{
		std::cout << outindices[i] << ",";
	}
	std::cout << std::endl;
}

std::vector<wp::Cutout> genAllOrientedCutouts(const wp::Cutout& co,const std::vector<wp::balltransform2f>& orientations)
{
	std::vector<wp::Cutout> allcutouts;
	for(size_t i=0;i<orientations.size();i++)
	{
		allcutouts.push_back(co);
		allcutouts.back().transform_in_place(orientations[i]);
		wp::balltransform2f offset(-allcutouts.back().mesh.bounding_box.min());
		allcutouts.back().transform_in_place(offset);
	}
	return allcutouts;
}

std::vector<wp::balltransform2f> genAllOrientations(size_t N,bool allow_inversions)
{
	std::vector<size_t> sdb=split_dominant_bisect(N);
	float angleDelta=2.0*M_PI/static_cast<float>(N);
	std::vector<wp::balltransform2f> orientations_out;
	Eigen::Matrix2f flipmat=Eigen::Matrix2f::Identity();
	flipmat(0,0)=-1.0f;
	
	for(size_t i=0;i<sdb.size();i++)
	{
		wp::balltransform2f rot;
		float angle=static_cast<float>(sdb[i])*angleDelta;
		std::cerr << "angle: " << angle << std::endl;
		rot.rotation=Eigen::Rotation2D<float>(angle).toRotationMatrix();
		orientations_out.push_back(rot);
		if(allow_inversions)
		{
			rot.rotation=rot.rotation*flipmat;
			orientations_out.push_back(rot);
		}
	}
	return orientations_out;
}

void splitDBtest()
{
	std::vector<wp::balltransform2f> orientations=genAllOrientations(4,true);
	for(size_t i=0;i<orientations.size();i++)
	{
		std::cerr << orientations[i].rotation << std::endl <<"EEEEEE" <<std::endl;
	}
}

int main(int argc,char** argv)
{
	//randomWeightTest();
	splitDBtest();
	return 0;
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

