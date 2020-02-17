#include "Cutout.hpp"
using namespace wp;

static ball2f aabb_parentfunc(
	const ball2f& lref,
	const ball2f& rref,
	const ball2f* bb,
	const ball2f* be,
	bool& swaplr
)
{
	
	swaplr=lref.position.x() > rref.position.x();
	Eigen::AlignedBox2f bbox=bb[0].leaf.bounds();
	size_t n=be-bb;
	//std::cerr << "The N: " << n << std::endl;
	//std::cerr << bbox.min().transpose() << ":" << bbox.max().transpose() << std::endl;
	for(size_t i=1;i<n;i++)
	{
		bbox.extend(bb[i].leaf.bounds());
		//	std::cerr << bbox.min().transpose() << ":" << bbox.max().transpose() << std::endl;
	}
	
	ball2f parent;
	if(swaplr)
	{
		parent=balltree2f::ball(rref,lref);
	}
	else
	{
		parent=balltree2f::ball(lref,rref);
	}
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
static balltree2f build_btree(const std::vector<wp::Triangle>& tris)
{
	std::vector<ball2f> all_balls;
	for(size_t i=0;i<tris.size();i++)
	{
		Eigen::Vector3f ecirc=tris[i].smallest_enclosing_circle();
		all_balls.emplace_back(tris[i],Eigen::Vector2f(ecirc.x(),ecirc.y()),ecirc.z());
	}
	
	balltree2f bt(all_balls.data(),all_balls.data()+all_balls.size(),aabb_parentfunc,0);
	return bt;
}
Cutout::Cutout(const wp::Mesh& msh):
	mesh(msh),
	tree(build_btree(mesh.triangles))
{}
void wp::Cutout::transform_in_place(const balltransform2f& tform)
{
	Eigen::Matrix<float,2,3> mtform=tform.pmatrix();
	mesh.transform_in_place(mtform);
	for(size_t i=0;i<tree.allnodes.size();i++)
	{
		tree.allnodes[i].transform_in_place(tform);
		tree.allnodes[i].leaf.transform_in_place(mtform);
	}
	tform_tracker=tform*tform_tracker;
}

void wp::Cutout::merge_in_place(const Cutout& other,const balltransform2f& tform)
{
	wp::Mesh mshtmp=other.mesh;
	mshtmp.transform_in_place(tform.pmatrix());
	mesh.merge_in_place(mshtmp);
	tree=build_btree(mesh.triangles);
}

