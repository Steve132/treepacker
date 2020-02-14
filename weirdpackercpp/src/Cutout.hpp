#ifndef CUTOUT_HPP
#define CUTOUT_HPP

#include "Triangle.hpp"
#include "BallTree.hpp"
#include "Mesh.hpp"

namespace wp
{
typedef balltree<wp::Triangle,2,float> balltree2f;
typedef typename balltree<wp::Triangle,2,float>::ball ball2f;
typedef balltransform<2,float> balltransform2f;

//a cutout is essentially a balltree with a mesh bounding box, current transform, and merge ability


class Cutout
{
public:
	
	wp::Mesh mesh;
	balltree2f tree;
	
	Cutout(const wp::Mesh& msh);
	
	bool intersect(const Cutout& other,const balltransform2f& tform=balltransform2f()) const;
	void transform_in_place(const balltransform2f& tform);
	void merge_in_place(const Cutout& other,const balltransform2f& tform);
};
	
}
#include "Cutout.inl"
#endif
