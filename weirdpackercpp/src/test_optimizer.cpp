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
//#include <gperftools/profiler.h>
#include "CutoutSet.hpp"

#include "OptimizerCPU.hpp"
#include "Optimize.hpp"
#include <random>

int main(int argc,char** argv)
{
	// initial problem definition example copied from test_cutoutpacking.cpp:
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

	// initialize optimizestate candidate, problemdefinition:
	wp::Candidate cand;
	cand.score = std::numeric_limits<float>::max(); // or do_score(ntest,testbb.sizes(),ul);
	/* Not sure how to initialize these, but since they're not used in OptimizerCPU, I'm not setting them for now, but you probably should!~~~~~
	cand.part_selection_indices = std::vector<size_t>; // ????
	cand.orientation = std::vector<balltransform2f>; // ????? test_cutoutpacking.cpp has balltransform2f as translations, rotations etc.
	cand.bounds = Eigen::Vector2f; // IDK, whatever holds all the meshes??? what are some mesh values? for coordinate space
	*/

	wp::ProblemDefinition pd = {
		cutouts,
		Eigen::Vector2f(794,1123), //table_bounds. assuming pixels here? //drawing.bounds()); // ????
		std::vector<int>(1), //num_angles_per_cutout ????
		Eigen::Vector2f(.3,.3) //increment_float: should be ~ .5mm to pixels or whatever the resolution of the cutting is
	};

	//std::random_device rd;
	wp::OptimizeState os = {
		std::mt19937_64(),//std::default_random_engine { rd }, // engine: used in std::shuffle(yvals.begin(),yvals.end(),state.engine); and select_indices where it operates on the vector<int> out: ordered_sample_without_replacement_indices(allcutoutscache.average_box_weights.begin(), allcutoutscache.average_box_weights.end(), ncutouts, state.engine, out.begin());
		cand, // current_best
		pd
	};

	// start the processing for brute-forcing packing:
	wp::OptimizerCPU optimizer(os);
	optimizer.next(0);
	return 0;
}
