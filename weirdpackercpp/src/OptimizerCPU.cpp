#include "OptimizerCPU.hpp"
#include <limits>
#include "SampleWithoutReplacement.hpp"
#include <Eigen/Dense>
void wp::OptimizerCPU::build_cs(const ProblemDefinition& pd)
{
	allcutoutscache=CutoutSet();
	for(size_t i=0;i<pd.cutouts.size();i++)
	{
		int norients=pd.num_angles_per_cutout[i];
		allcutoutscache.add(pd.cutouts[i],wp::CutoutSet::generate_orientations(std::abs(norients),norients < 0));
	}
}
wp::OptimizerCPU::OptimizerCPU(const OptimizeState& tstate):
	OptimizerBase(tstate)
{
	build_cs(tstate.pd);
	state.current_best.score=std::numeric_limits<float>::max();
	size_t Ny=2+state.pd.table_bounds.max().y()/state.pd.increment_float.y();
	yvals.resize(Ny);
}

void wp::OptimizerCPU::select_indices(std::vector<size_t>& out)
{
	size_t ncutouts=state.pd.cutouts.size();
	if(out.size() != ncutouts)
	{
		out.resize(ncutouts);
	}
	ordered_sample_without_replacement_indices(
		allcutoutscache.average_box_weights.begin(),
		allcutoutscache.average_box_weights.end(),
		ncutouts,
		state.engine,
		out.begin());
	
}

static inline float do_score(const Eigen::Vector2f& ntest,const Eigen::Vector2f& testbb,const Eigen::Vector2f& ul)
{
	return ntest.array().max((testbb+ul).array()).prod(); //minimize L1
}

void wp::OptimizerCPU::place_one(Cutout& table,size_t part_id)
{
	const std::vector<Cutout>& rotations=allcutoutscache.cutouts[part_id];

	Eigen::Vector2f ntest(table.mesh.bounding_box.sizes().x(),state.pd.table_bounds.max().y());
	Eigen::Vector2f incg=state.pd.increment_float;
	ntest.x()+=2.0f*incg.x();
	const size_t Ny=yvals.size();
	
	for(size_t i=0;i<Ny;i++) yvals[i]=incg.y()*i;
	std::shuffle(yvals.begin(),yvals.end(),state.engine);
	
	float global_cscore=std::numeric_limits<float>::max();
	
	size_t best_global_rotation=0;
	wp::balltransform2f best_global_offset(ntest);

	#pragma omp parallel for schedule(dynamic)
	for(size_t yi=0;yi<Ny;yi++)
	{
		const Eigen::Vector2f incloc=incg;
		float local_cscore=global_cscore;
		Eigen::Vector2f ul(0.0f,yvals[yi]);
		wp::balltransform2f best_local_offset=ul;
		size_t best_local_rotation=0;
		
		bool any_found=false;
		
		for(size_t ri=0;ri<rotations.size();ri++)
		{
			const Cutout& test=rotations[ri];
			Eigen::AlignedBox2f testbb=test.mesh.bounding_box;
			if(testbb.sizes().y()+ul.y() > ntest.y()) continue;
			
			for(ul.x()=0.0;ul.x() < ntest.x();ul.x()+=incloc.x())
			{			
				wp::balltransform2f offset(ul);
				float nscore=do_score(ntest,testbb.sizes(),ul);
				if(nscore < local_cscore)
				{
					if(!table.intersect(test,offset))
					{
						local_cscore=nscore;
						best_local_offset=offset;
						best_local_rotation=ri;
						any_found=true;
					}
				}
				else
				{
					break; //is this a speedup?
				}
			}
		}
		if(any_found)
		{
			#pragma omp critical
			{
				if(local_cscore < global_cscore)
				{
					global_cscore=local_cscore;
					best_global_offset=best_local_offset;
					best_global_rotation=best_local_rotation;
				}
			}
		}
	}
	table.merge_in_place(rotations[best_global_rotation],best_global_offset);
}
//Todo thread all of this
void wp::OptimizerCPU::next(size_t extra)
{
	for(size_t i=0;i<extra;i++)
	{
		next(0);
	}
	
	Candidate cur;
	select_indices(cur.part_selection_indices);
	
	Cutout table;//(cur.part_selection_indices[0]); //TODO weird issue:first part shows up in upper left in no rotation every time.
	for(size_t placement_stage=0;cur.part_selection_indices.size();placement_stage++)
	{
		place_one(table,cur.part_selection_indices[placement_stage]);
	}
	cur.score=table.mesh.bounding_box.sizes().x();
	if(cur.score < state.current_best.score)
	{
		//convert rotation indices back into angles and fold into offsets, update best candidate
	}
}
