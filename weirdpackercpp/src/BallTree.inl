#include<Eigen/Geometry>

template<unsigned int D,class REAL>
inline 
ballbase<D,REAL>::ballbase(const Eigen::Matrix<REAL,D,1>& pos,REAL trad):
	position(pos),
	radius(trad),
	num_children(0)
{}

template<unsigned int D,class REAL>
inline 
ballbase<D,REAL>::ballbase(const ballbase& lball,const ballbase& rball):
	num_children(lball.num_children+rball.num_children+2)
{
	Eigen::Matrix<REAL,D,1> pos_mean,childaxis;
	childaxis=rball.position-lball.position;
	REAL mg=childaxis.norm();
	if(mg <= 0.0000001)
	{
		childaxis=Eigen::Matrix<REAL,D,1>::Zero();
		childaxis[0]=static_cast<REAL>(1.0);
	}
	else
	{
		childaxis/=mg;
	}
	Eigen::Matrix<REAL,D,1> lower=lball.position-lball.radius*childaxis;
	Eigen::Matrix<REAL,D,1> upper=rball.position+rball.radius*childaxis;
	radius=(upper-lower).norm()*static_cast<REAL>(0.5);
	position=(upper+lower)*static_cast<REAL>(0.5);
}	
template<unsigned int D,class REAL>
inline 
void ballbase<D,REAL>::transform_in_place(const balltransform<D,REAL>& Rt)
{
	position=Rt.scale*Rt.rotation*position;
	position+=Rt.translation;
	radius*=Rt.scale;
}
template<unsigned int D,class REAL>
inline 
bool ballbase<D,REAL>::intersect(const ballbase& b) const
{
	double r=(b.radius+radius);
	return (b.position-position).squaredNorm() < r*r;
}

template<class LeafType,unsigned int D,class REAL>
template<class LeafBallType2,class LeafIntersectFunc>
inline
bool balltree<LeafType,D,REAL>::ball::intersect(const LeafBallType2& other,LeafIntersectFunc leaf_intersect) const
{
	if(ballbase<D,REAL>::num_children || other.num_children)
	{
		return ballbase<D,REAL>::intersect(other);
	}
	
	return leaf_intersect(leaf,other.leaf);
}

namespace 
{
template<class VectorType,class MatrixType> 
static inline
typename std::enable_if<MatrixType::RowsAtCompileTime <= 3>::type
getDomEigenvector(VectorType& vout,const MatrixType& A)
{
	Eigen::SelfAdjointEigenSolver<MatrixType> saes;
	vout=saes.computeDirect(A).eigenvectors().col(A.cols()-1);
}
template<class VectorType,class MatrixType> 
static inline
typename std::enable_if<(MatrixType::RowsAtCompileTime > 3)>::type
getDomEigenvector(VectorType& vout,const MatrixType& A)
{
	Eigen::SelfAdjointEigenSolver<MatrixType> saes;
	vout=saes.compute(A).eigenvectors().col(A.cols()-1);
}

}
template<class LeafType,unsigned int D,class REAL>
inline balltree<LeafType,D,REAL>::ball::ball(){}

template<class LeafType,unsigned int D,class REAL>
inline balltree<LeafType,D,REAL>::ball::ball(const LeafType& lt,const Eigen::Matrix<REAL,D,1>& pos,REAL trad): //its illegal to make an empty leaf node
	ballbase<D,REAL>(pos,trad),
	leaf(lt)
	{}
	
template<class LeafType,unsigned int D,class REAL>
	inline balltree<LeafType,D,REAL>::ball::ball(const ballbase<D,REAL>& a,const ballbase<D,REAL>& b,const LeafType& lt):
		ballbase<D,REAL>(a,b),
		leaf(lt)
	{}
template<class LeafType,unsigned int D,class REAL>
template<class MakeParentFunc>
inline std::forward_list<typename balltree<LeafType,D,REAL>::ball> balltree<LeafType,D,REAL>::build_balltree_dfs(ball* bbegin,ball* bend,MakeParentFunc makeparent,size_t extra_leaf_size)
{
	size_t n=bend-bbegin;
	extra_leaf_size=0;//TODO: this parameter doesn't work so disable it
	if(n<=(extra_leaf_size+1))
	{
		std::forward_list<ball> lo;
		for(size_t k=0;k<(extra_leaf_size+1);k++)
		{
			lo.push_front(*bbegin);
		}
		return lo;
	}
	Eigen::Matrix<REAL,D,1> pos_mean,pos_sum=Eigen::Matrix<REAL,D,1>::Zero();
	Eigen::Matrix<REAL,D,D> pos_lcov=Eigen::Matrix<REAL,D,D>::Zero();
	for(const ball* bi=bbegin;bi != bend;bi++)
	{
		Eigen::Matrix<REAL,D,1> epos=bi->position;
		pos_sum+=epos;
		pos_lcov+=epos*epos.transpose();
	}
	REAL nf=static_cast<REAL>(n);
	pos_mean=pos_sum/nf;
	pos_lcov-=pos_sum*pos_mean.transpose();
	Eigen::Matrix<REAL,D,1> childaxis;
	::getDomEigenvector(childaxis,pos_lcov);
	
	std::for_each(bbegin,bend,[childaxis,pos_mean](ball& b)
	{
		b.temp_projector=(b.position-pos_mean).dot(childaxis);
	});
	ball* bmid=bbegin+n/2;
	std::nth_element(bbegin,bmid,bend,[](const ball& a,const ball& b)
	{
		return a.temp_projector < b.temp_projector;
	});
	auto left_list=build_balltree_dfs(bbegin,bmid,makeparent,extra_leaf_size);
	auto right_list=build_balltree_dfs(bmid,bend,makeparent,extra_leaf_size);
	bool swaplr=false;
	ball newroot=makeparent(left_list.front(),right_list.front(),bbegin,bend,swaplr);

	if(swaplr)
	{
		std::swap(right_list,left_list);
	}
	right_list.splice_after(right_list.before_begin(),left_list,left_list.before_begin(),left_list.end());
	right_list.push_front(newroot);
	return right_list;
}

template<class LeafType,unsigned int D,class REAL>
template<class MakeParentFunc>
inline balltree<LeafType,D,REAL>::balltree(ball* bbegin,ball* bend,MakeParentFunc makeparent,size_t extra_leaf_size)
{
	std::forward_list<ball> ball_list=build_balltree_dfs(bbegin,bend,makeparent,extra_leaf_size); 
	allnodes=std::vector<ball>(ball_list.begin(),ball_list.end());
}
template<class LeafType,unsigned int D,class REAL>
inline balltree<LeafType,D,REAL>::balltree(ball* bbegin,ball* bend):
	balltree(bbegin,bend,[](const ball& l,const ball& r,const ball* innerbb,const ball* innerbe,bool&)
	{
		return ball(l,r);
	})
{
}

template<class LeafType,unsigned int D,class REAL>
template<class ClientIntersectFunc>
inline bool balltree<LeafType,D,REAL>::intersect_callable(ClientIntersectFunc client_intersect_func,size_t iroot) const
{
	for(size_t root=iroot;root < allnodes.size();root++)
	{
		const ball& thisnode=allnodes[root];
		if(client_intersect_func(thisnode))
		{
			if(thisnode.num_children==0) 
			{
				return true;
			}
		}
		else
		{
			root+=thisnode.num_children;
		}
	}
	return false;
}
template<class LeafType,unsigned int D,class REAL>
template<class LeafBallType,class LeafIntersectFunc>
inline bool balltree<LeafType,D,REAL>::intersect(const LeafBallType& b,LeafIntersectFunc leaf_intersect,size_t root) const
{
	
	auto client_func=[&b,&leaf_intersect](const ball& other)
	{
		return other.intersect(b,leaf_intersect);
	};
	return intersect_callable(client_func,root);
}

