#include "OptimizerCPU.hpp"

wp::OptimizerCPU::OptimizerCPU(const OptimizeState& tstate):
	OptimizerBase(tstate)
{
	build_cs(tstate.pd);
}


