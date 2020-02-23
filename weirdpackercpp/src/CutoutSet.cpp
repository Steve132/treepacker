#include "CutoutSet.hpp"

static std::vector<size_t> split_dominant_bisect(size_t N)
{
	std::vector<size_t> order(N,N);
	size_t position=0;
	for(size_t increment=N/2;increment > 0;increment/=2)
	{
		for(size_t k=0;k<N;k+=increment)
		{
			if(order[k]==N)
			{
				order[k]=position++;
			}
		}
	}
	std::vector<size_t> output(N);
	for(size_t k=0;k<N;k++)
	{
		size_t v=order[k];
		if(v != N)
		{
			output[v]=k;
		}
	}
	return output;
}

std::vector<wp::balltransform2f> wp::CutoutSet::generate_orientations(size_t N,bool allow_inversions)
{
	std::vector<size_t> sdb=split_dominant_bisect(N);
	float angleDelta=2.0*M_PI/static_cast<float>(N);
	std::vector<wp::balltransform2f> orientations_out;
	Eigen::Matrix2f flipmat=Eigen::Matrix2f::Identity();
	flipmat(0,0)=-1.0f;
	
	for(size_t i=0;i<sdb.size();i++)
	{
		wp::balltransform2f rot;
		float angle=static_cast<float>(sdb[i])*angleDelta;
		std::cerr << "angle: " << angle << std::endl;
		rot.rotation=Eigen::Rotation2D<float>(angle).toRotationMatrix();
		orientations_out.push_back(rot);
		if(allow_inversions)
		{
			rot.rotation=rot.rotation*flipmat;
			orientations_out.push_back(rot);
		}
	}
	return orientations_out;
}
void wp::CutoutSet::add(const wp::Cutout& co,const std::vector<wp::balltransform2f>& orientations)
{
	std::vector<wp::Cutout> allcutouts;
	double avg_boxweight=0.0;
	for(size_t i=0;i<orientations.size();i++)
	{
		allcutouts.push_back(co);
		allcutouts.back().transform_in_place(orientations[i]);
		wp::balltransform2f offset(-allcutouts.back().mesh.bounding_box.min());
		allcutouts.back().transform_in_place(offset);
		avg_boxweight+=allcutouts[i].mesh.bounding_box.sizes().array().prod();
	}
	avg_boxweight/=orientations.size();
	cutouts.push_back(allcutouts);
	average_box_weights.push_back(avg_boxweight);
}
