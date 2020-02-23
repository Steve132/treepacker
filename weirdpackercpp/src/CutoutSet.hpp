#ifndef CUTOUTSET_HPP
#define CUTOUTSET_HPP

#include "Cutout.hpp"
#include <vector>

namespace wp
{

class CutoutSet
{
public:
	std::vector<std::vector<Cutout> > cutouts;
	std::vector<double> average_box_weights;
public:
	static std::vector<balltransform2f> generate_orientations(size_t N=1,bool allow_flip=false);
	void add(const Cutout& co,const std::vector<balltransform2f>& rotations);
};
}
#endif
