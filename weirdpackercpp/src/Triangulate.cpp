#include "Triangulate.hpp"
#include "poly2tri/poly2tri.h"
#include<memory>
#include<algorithm>


static std::vector<p2t::Point*> polyline_convert(const std::vector<wp::Point>& l)
{
	std::vector<p2t::Point*> p2o(l.size());
	for(size_t i=0;i<l.size();i++)
	{
		p2o[i]=new p2t::Point(l[i][0],l[i][1]);
	}
	return p2o;
}
static std::vector<wp::Point> polyline_backconvert(const std::vector<p2t::Point*>& pref)
{
	std::vector<wp::Point> p2o(pref.size());
	for(size_t i=0;i<pref.size();i++)
	{
		p2o[i]=wp::Point(pref[i]->x,pref[i]->y);
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
			wp::Point{points[0]->x,points[0]->y},
			wp::Point{points[1]->x,points[1]->y},
			wp::Point{points[2]->x,points[2]->y}
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
std::vector<wp::Triangle> wp::triangulate(const Shape& shape)
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
