#include<iostream>
#include "Shape.hpp"
#include<iterator>
#include<string>
#include<iterator>
#include<algorithm>
#include<fstream>
#include "Bezier.hpp"
#include<stdexcept>
using namespace std;

struct PathCommand
{
	char command_char;
	std::vector<double> command_args;
};

struct SubPath
{
public:
	std::vector<Eigen::Vector2f> segments;
	SubPath(const Eigen::Vector2f& beg):segments{beg}
	{}
	bool empty() const
	{
		return segments.size() == 1;
	}
};
static inline Eigen::Vector2f relorabs(const Eigen::Vector2f& current,Eigen::Vector2f offset,bool isrel)
{
	if(isrel) offset+=current;
	return offset;
}

struct SegmentsContext
{
	std::vector<SubPath> subpaths;
	SubPath current_subpath;
	Eigen::Vector2f cursor;
	double points_per_unit;
	
	Eigen::Vector2f previous_control;
	int previous_bezier_command; //0 means none, 2 means quadratic, 3 means cubic 
	
	SegmentsContext(double ppu):
		current_subpath(Eigen::Vector2f::Zero()),
		cursor(Eigen::Vector2f::Zero()),
		points_per_unit(ppu),
		previous_control(Eigen::Vector2f::Zero()),
		previous_bezier_command(0)
	{}
	void executePathCommand(const PathCommand& pc);
	void reset_bezier_control(int pbc=0)
	{
		previous_bezier_command=pbc;
	}
	void endSubpath()
	{
		if(!current_subpath.empty())
		{
			subpaths.push_back(current_subpath);
		}
		current_subpath=SubPath(cursor);
		reset_bezier_control(0);
	}
	void absMoveTo(const Eigen::Vector2f& dst)
	{
		cursor=dst;
		endSubpath();
		reset_bezier_control(0);
	}
	void absLineToBase(const Eigen::Vector2f& dst)
	{
		cursor=dst;
		current_subpath.segments.push_back(cursor);
	}
	void absLineTo(const Eigen::Vector2f& dst)
	{
		absLineToBase(dst);
		reset_bezier_control(0);
	}
	void absBeizer3To(const Eigen::Vector2f& c1,const Eigen::Vector2f& c2,const Eigen::Vector2f& endpoint)
	{
		Bezier<3,Eigen::Vector2f> bz({cursor,c1,c2,endpoint});
		previous_control=c2;
		reset_bezier_control(3);
		
		size_t n=bz.estimate_num_segments(points_per_unit);
		float df=static_cast<float>(1.0f)/static_cast<float>(n);
		for(size_t i=0;i<n;i++)
		{
			absLineToBase(bz.interpolate(static_cast<float>(i)*df));
		}
		cursor=endpoint;
	}
	void absBeizer2To(const Eigen::Vector2f& c1,const Eigen::Vector2f& endpoint)
	{
		Bezier<2,Eigen::Vector2f> bz({cursor,c1,endpoint});
		previous_control=c1;
		reset_bezier_control(2);
		
		size_t n=bz.estimate_num_segments(points_per_unit);
		float df=static_cast<float>(1.0f)/static_cast<float>(n);
		for(size_t i=0;i<n;i++)
		{
			absLineToBase(bz.interpolate(static_cast<float>(i)*df));
		}
		
		cursor=endpoint;
	}
	void absArcTo(const Eigen::Vector2f& radius,float angle,bool large_arc,bool sweep,const Eigen::Vector2f& endpoint)
	{
		cursor=endpoint;
		reset_bezier_control(0);
		throw std::invalid_argument("Arcs are currently not supported by pathing library");
	}
};

void SegmentsContext::executePathCommand(const PathCommand& pc)
{
	bool iL=std::islower(pc.command_char);
	bool isH=false;
	switch(pc.command_char)
	{
		//Subpath Splits
		case 'm':
		case 'M':
		{
			Eigen::Vector2f dst(pc.command_args[0],pc.command_args[1]);
			absMoveTo(relorabs(cursor,dst,iL));
			for(size_t i=1;i<pc.command_args.size()/2;i++)
			{
				Eigen::Vector2f dst2(pc.command_args[2*i],pc.command_args[2*i+1]);
				absLineTo(relorabs(cursor,dst2,iL));
			}
			break;
		}
		case 'z':
		case 'Z':
		{
			if(!current_subpath.empty())
			{
				Eigen::Vector2f dst=current_subpath.segments.front();
				absLineTo(dst);
				absMoveTo(dst);
			}
			break;
		}
		//Lines
		case 'l':
		case 'L':
		{
			for(size_t i=0;i<pc.command_args.size() / 2; i++)
			{
				Eigen::Vector2f dst(pc.command_args[2*i],pc.command_args[2*i+1]);
				absLineTo(relorabs(cursor,dst,iL));
			}
			break;
		}
		case 'h':
		case 'H':
			isH=true;
		case 'v':
		case 'V':
		{
			for(size_t i=0;i<pc.command_args.size();i++)
			{
				Eigen::Vector2f dst(0.0f,pc.command_args[i]);
				if(isH) 
				{
					std::swap(dst.x(),dst.y());
				}
				dst=relorabs(cursor,dst,iL);
				if(isH)
				{
					dst.y()=cursor.y();
				}
				else
				{
					dst.x()=cursor.x();
				}
				absLineTo(dst);
			}
			break;
		}
		//Cubic Bezier
		case 'c':
		case 'C':
		{
			for(size_t i=0;i<pc.command_args.size()/6;i++)
			{
				Eigen::Vector2f c1(pc.command_args[6*i+0],pc.command_args[6*i+1]);
				Eigen::Vector2f c2(pc.command_args[6*i+2],pc.command_args[6*i+3]);
				Eigen::Vector2f ep(pc.command_args[6*i+4],pc.command_args[6*i+5]);
				c1=relorabs(cursor,c1,iL);
				c2=relorabs(cursor,c2,iL);
				ep=relorabs(cursor,ep,iL);
				absBeizer3To(c1,c2,ep);
			}
			break;
		}
		case 's':
		case 'S':
		{
			for(size_t i=0;i<pc.command_args.size()/4;i++)
			{
				Eigen::Vector2f c1=cursor;
				Eigen::Vector2f c2(pc.command_args[4*i+0],pc.command_args[4*i+1]);
				Eigen::Vector2f ep(pc.command_args[4*i+2],pc.command_args[4*i+3]);
				c2=relorabs(cursor,c2,iL);
				ep=relorabs(cursor,ep,iL);
				if(previous_bezier_command==3)
				{
					c1-=(previous_control-cursor);
				}
				absBeizer3To(c1,c2,ep);
			}
			break;
		}
		//Quadratic Bezier
		case 'q':
		case 'Q':
		{
			for(size_t i=0;i<pc.command_args.size()/4;i++)
			{
				Eigen::Vector2f c1(pc.command_args[4*i+0],pc.command_args[4*i+1]);
				Eigen::Vector2f ep(pc.command_args[4*i+2],pc.command_args[4*i+3]);
				c1=relorabs(cursor,c1,iL);
				ep=relorabs(cursor,ep,iL);
				absBeizer2To(c1,ep);
			}
			break;
		}
		case 't':
		case 'T':
		{
			for(size_t i=0;i<pc.command_args.size()/2;i++)
			{
				Eigen::Vector2f c1=cursor;
				Eigen::Vector2f ep(pc.command_args[2*i+2],pc.command_args[2*i+3]);
				ep=relorabs(cursor,ep,iL);
				if(previous_bezier_command==2)
				{
					c1-=(previous_control-cursor);
				}
				absBeizer2To(c1,ep);
			}
			break;
		}
		case 'a':
		case 'A':
		{
			for(size_t i=0;i<pc.command_args.size()/7;i++)
			{
				Eigen::Vector2f rad(pc.command_args[7*i+0],pc.command_args[7*i+1]);
				double angle=pc.command_args[7*i+2];
				bool large_arc=pc.command_args[7*i+3] > 0.5;
				bool sweep=pc.command_args[7*i+4] > 0.5;
				Eigen::Vector2f ep(pc.command_args[7*i+5],pc.command_args[7*i+6]);
				ep=relorabs(cursor,ep,iL);
				absArcTo(rad,angle,large_arc,sweep,ep);
			}
			break;
		}
	};
	
}


std::istream& operator>>(std::istream& is,PathCommand& cmd)
{
	is >> cmd.command_char;
	cmd.command_args.clear();
	while(is.good())
	{
		int nc=is.peek();
		if(std::isspace(nc) || nc==',')
		{
			is.ignore(1);
		}
		else if(std::isalpha(nc))
		{
			break;
		}
		else
		{
			double a;
			is >> a;
			cmd.command_args.push_back(a);
		}
	}
	return is;
}
std::ostream& operator<<(std::ostream& out,const PathCommand& cmd)
{
	out << cmd.command_char;
	out << ":";
	std::copy(cmd.command_args.begin(),cmd.command_args.end(),std::ostream_iterator<double>(out," "));
	return out;
}

namespace trail
{
trail::Shape path2shape(std::istream& inp,const std::string& name,double points_per_unit)
{
	auto CommandIterBegin=std::istream_iterator<PathCommand>(inp);
	auto CommandIterEnd=std::istream_iterator<PathCommand>();
	SegmentsContext sc(points_per_unit);
	
	std::for_each(CommandIterBegin,CommandIterEnd,[&sc](const PathCommand& pc)
		{
			sc.executePathCommand(pc);
		}
	);
	sc.endSubpath();
	trail::Shape sp;
	sp.id=name;
	sp.outerline=sc.subpaths[0].segments;
	if(sc.subpaths.size())
	{
		sp.holes.resize(sc.subpaths.size()-1);
		for(size_t i=0;i<sp.holes.size();i++)
		{
			sp.holes[i]=sc.subpaths[i+1].segments;
		}
	}
	return sp;
}
}
