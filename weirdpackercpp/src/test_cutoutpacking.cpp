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
	std::cout << rengine << std::endl;
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

wp::balltransform2f find_first_fit(const wp::Cutout& table,const wp::Cutout& test,const Eigen::Vector2f& increment,const Eigen::Vector2f& bounds)
{
	Eigen::AlignedBox2f tbb=test.mesh.bounding_box;
	Eigen::Vector2f ntest=(bounds-tbb.sizes());
	ntest=ntest.array().min((table.mesh.bounding_box.max()+increment*2.0f).array());
	Eigen::Vector2f ul=Eigen::Vector2f::Zero();
	float cscore=bounds.prod();
	wp::balltransform2f best_offset=ul;
	//add a branch/bound step here where if you are outside the bounding box or worse than the current best (like if you're at the end of the x or the y) then you just bail.
	for(ul.y()=0.0f;ul.y()<ntest.y();ul.y()+=increment.y()) //this doesn't work because we actually want to find the one that grows the overall bounding box the least.
	for(ul.x()=0.0f;ul.x()<ntest.x();ul.x()+=increment.x())
	{
		wp::balltransform2f offset(ul);
		if(!table.intersect(test,offset))
		{
			float score=table.mesh.bounding_box.max().array().max((tbb.max()+ul).array()).prod();
			if(score < cscore)
			{
				cscore=score;
				best_offset=offset;
			}
			break;
		}
	}
	return best_offset;
}

int main(int argc,char** argv)
{
	randomWeightTest();
	//splitDBtest();
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
	
	int selected=0;
	std::vector<wp::Cutout> oriented_cutouts=genAllOrientedCutouts(cutouts[1],genAllOrientations(90,false));
	
	wp::balltransform2f mouse_tform;
	auto start=std::chrono::high_resolution_clock::now();
	static const size_t N=10000;
	#pragma omp parallel for
	for(size_t i=0;i<N;i++)
	{
		mouse_tform=find_first_fit(cutouts[0],oriented_cutouts[selected],Eigen::Vector2f(0.01,0.01),Eigen::Vector2f(5.0f,3.0f));
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
			selected=(selected+1) % oriented_cutouts.size();
			//ProfilerStart("nameOfProfile.log");
			
			//ProfilerStop();
		}
		//std::cout << Rt2 << std::endl;
		wp::Cutout temp_cut2=oriented_cutouts[selected];
		temp_cut2.transform_in_place(mouse_tform);
		
		bool inter=cutouts[0].intersect(oriented_cutouts[selected],mouse_tform);
		
		uint8_t cv=inter ? 0xFF : 0x00;
		r.draw(temp_cut2.mesh,{cv,~cv,0x00},true);
		r.update();
	}
	
	return 0;
}

