#ifndef OPTIMIZE_HPP
#define OPTIMIZE_HPP

#include<random>
#include<Eigen/Geometry>
#include<Eigen/Dense>
#include<iostream>
#include "Cutout.hpp"

namespace wp
{
	
	class Candidate
	{
		float score;
		std::vector<size_t> part_selection_indices;
		std::vector<balltransform2f> orientation;
	};
	class ProblemDefinition
	{
		std::vector<Cutout> cutouts;
		Eigen::AlignedBox2f table_bounds;
		std::vector<size_t> num_angles_per_cutout;
		Eigen::Vector2f increment_float;
	};
	
	class OptimizeState
	{
	public:
		std::mt19937_64 engine; //standardize on mt19937_64 because we need a long period and it doesn't have to be hyper fast
		Candidate current_best;
		ProblemDefinition pd;
	};
	std::ostream& operator<<(std::ostream&,const OptimizeState&);
	std::istream& operator>>(std::istream&,OptimizeState&);
	
	class OptimizerBase
	{
	public:
		OptimizeState state;
		OptimizerBase(const OptimizeState& tstate=OptimizeState()):
			state(tstate)
		{}
		virtual ~OptimizerBase() {};
		virtual void next(size_t extra=0) =0; //always advances at least 1, advances extra+1 configs
	};
	
}

#endif

