#include "Mesh.hpp"
#include "poly2tri/poly2tri.h"
#include<memory>
#include<algorithm>
#include<numeric>


static std::vector<p2t::Point*> polyline_convert(const std::vector<trail::Point>& l)
{
	std::vector<p2t::Point*> p2o(l.size());
	for(size_t i=0;i<l.size();i++)
	{
		p2o[i]=new p2t::Point(l[i][0],l[i][1]);
	}
	return p2o;
}
static std::vector<trail::Point> polyline_backconvert(const std::vector<p2t::Point*>& pref)
{
	std::vector<trail::Point> p2o(pref.size());
	for(size_t i=0;i<pref.size();i++)
	{
		p2o[i]=trail::Point(pref[i]->x,pref[i]->y);
	}
	return p2o;
}
static std::vector<wp::Triangle> triangle_backconvert(const std::vector<p2t::Triangle*>& tref)
{
	std::vector<wp::Triangle> t2o(tref.size());
	for(size_t i=0;i<tref.size();i++)
	{
		p2t::Triangle& tiref=*tref[i];
		std::array<const p2t::Point*,3> points{tiref.GetPoint(0),tiref.GetPoint(1),tiref.GetPoint(2)};
		
		wp::Triangle tri{
			trail::Point{points[0]->x,points[0]->y},
			trail::Point{points[1]->x,points[1]->y},
			trail::Point{points[2]->x,points[2]->y}
		};
		t2o[i]=tri;
	}
	return t2o;
}
static void polyline_delete(std::vector<p2t::Point*>& pref)
{
	for(size_t i=0;i<pref.size();i++)
	{
		delete pref[i];
	}
	pref.clear();
}
std::vector<wp::Triangle> wp::Mesh::triangulate(const trail::Shape& shape)
{
	std::vector<p2t::Point*> outer_line=polyline_convert(shape.outerline);
	std::shared_ptr<p2t::CDT> pcdt(new p2t::CDT(outer_line));
	std::vector<std::vector<p2t::Point*>> hole_lines(shape.holes.size());
	for(size_t i=0;i<shape.holes.size();i++)
	{
		hole_lines[i]=polyline_convert(shape.holes[i]);
		pcdt->AddHole(hole_lines[i]);
	}
	pcdt->Triangulate();
	std::vector<wp::Triangle> output=triangle_backconvert(pcdt->GetTriangles());
	
	std::for_each(hole_lines.begin(),hole_lines.end(),polyline_delete);
	polyline_delete(outer_line);
	return output;
}


wp::Mesh::Mesh(const std::vector<wp::Triangle>& sh):
triangles(sh)
{
	if(triangles.size())
	{
		bounding_box=triangles[0].bounds();
		for(const Triangle& tri : triangles)
		{
			auto bb=tri.bounds();
			bounding_box=bounding_box.merged(bb);
			area+=tri.area();
		}
	}
}
void wp::Mesh::transform_in_place(const Eigen::Matrix<float,2,3>& Rt)
{
	if(triangles.size())
	{
		triangles[0]=Triangle::transform(Rt,triangles[0]);
		bounding_box=triangles[0].bounds();
		for(size_t i=1;i<triangles.size();i++)
		{
			Triangle& ntri=triangles[i];
			bounding_box.extend(ntri.p[0]);
			bounding_box.extend(ntri.p[1]);
			bounding_box.extend(ntri.p[2]);
		}
	}
}
wp::Mesh wp::Mesh::transform(const Eigen::Matrix<float, 2, 3>& Rt, const wp::Mesh& trishape)
{
	wp::Mesh newmesh(trishape);
	newmesh.transform_in_place(Rt);
	return newmesh;
}


