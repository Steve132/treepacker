#ifndef BEZIER_HPP
#define BEZIER_HPP

#include<array>

template<size_t N,class VectorType>
class Bezier
{
public:
	std::array<VectorType,N+1> controlpoints;
	Bezier(const std::array<VectorType,N+1>& tcp):controlpoints(tcp)
	{}
	VectorType interpolate(double mu) const;
	double average_segment_norm(size_t n) const;
	size_t estimate_num_segments(double norm_target,size_t start=100) const;
};

#include "Bezier.inl"

#endif
