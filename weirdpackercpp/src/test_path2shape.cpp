#include<iostream>
#include "Shape.hpp"
#include<iterator>
#include<string>
#include<iterator>
#include<algorithm>
#include<fstream>
#include "Bezier.hpp"
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
	void absLineTo(const Eigen::Vector2f& dst)
	{
		cursor=dst;
		current_subpath.segments.push_back(cursor);
		reset_bezier_control(0);
	}
	void absBeizer3To(const Eigen::Vector2f& c1,const Eigen::Vector2f& c2,const Eigen::Vector2f& endpoint)
	{
		Bezier<3,Eigen::Vector2f> bz({cursor,c1,c2,endpoint});
		previous_control=c2;
		reset_bezier_control(3);
		cursor=endpoint;
	}
	void absBeizer2To(const Eigen::Vector2f& c1,const Eigen::Vector2f& endpoint)
	{
		Bezier<2,Eigen::Vector2f> bz({cursor,c1,endpoint});
		previous_control=c1;
		reset_bezier_control(2);
		cursor=endpoint;
	}
};




void SegmentsContext::executePathCommand(const PathCommand& pc)
{
	bool iL=std::islower(pc.command_char);
	bool isH=false;
	switch(pc.command_char)
	{
		case 'm':
		case 'M':
		{
			Eigen::Vector2f dst(pc.command_args[0],pc.command_args[1]);
			absMoveTo(relorabs(cursor,dst,iL));
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
				Eigen::Vector2f dst(0.0,pc.command_args[0]);
				if(isH) std::swap(dst.x(),dst.y());
				absLineTo(relorabs(cursor,dst,iL));
			}
			break;
		}
		case 'c':
		case 'C':
		
	}
	
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


wp::Shape path2shape(std::istream& inp,double points_per_unit)
{
	auto CommandIterBegin=std::istream_iterator<PathCommand>(inp);
	auto CommandIterEnd=std::istream_iterator<PathCommand>();
	std::copy(CommandIterBegin,CommandIterEnd,std::ostream_iterator<PathCommand>(cout,"\n"));
	return wp::Shape();
}
int main(int argc,char** argv)
{
	std::ifstream inpf("../../../data/test.path");
	
	path2shape(inpf,0.01);
}
