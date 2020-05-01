#ifndef OPTIMIZER_CPU_HPP
#define OPTIMIZER_CPU_HPP

#include "Optimize.hpp"
#include "CutoutSet.hpp"

namespace wp
{
	class OptimizerCPU: public OptimizerBase
	{
	private:
		CutoutSet allcutoutscache;
		std::vector<float> yvals;
		void select_indices(std::vector<size_t>& out);
		void build_cs(const ProblemDefinition& pd);
		void place_one(const Cutout& table,size_t part_id,size_t& rotation_out,wp::balltransform2f& transform_out);
	public:
		OptimizerCPU(const OptimizeState& os);
		virtual void next(size_t extra=0);
	};
}

#endif
