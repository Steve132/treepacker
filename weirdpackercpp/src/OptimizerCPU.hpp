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
		void build_cs(const ProblemDefinition& pd);
	public:
		OptimizerCPU(const OptimizeState& os);
		virtual void next(size_t extra=0);
	};
}

#endif
