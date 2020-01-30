#ifndef BALLTREE_HPP
#define BALLTREE_HPP

#include<Eigen/Dense>
#include<Eigen/Eigenvalues>
#include<algorithm>
#include<vector>
#include<forward_list>
#include<type_traits>

template<class LeafType,unsigned int D,class REAL=double>
class balltree
{
public:
	struct ball
	{
		Eigen::Matrix<REAL,D,1> position;
		REAL radius;
		LeafType leaf;
		size_t num_children;
		REAL temp_projector;
        ball(const LeafType& lt=LeafType(),REAL trad=0.0,const Eigen::Matrix<REAL,D,1>& pos=Eigen::Matrix<REAL,D,1>::Zero()):
			position(pos),
			radius(trad),
			leaf(lt),
			num_children(0)
        {}
        ball(const ball& lball,const ball& rball):num_children(lball.num_children+rball.num_children+2)
		{
			Eigen::Matrix<REAL,D,1> pos_mean,childaxis;
			childaxis=rball.position-lball.position;
			REAL mg=childaxis.norm();
			childaxis/=mg;
			Eigen::Matrix<REAL,D,1> lower=lball.position-lball.radius*childaxis;
			Eigen::Matrix<REAL,D,1> upper=rball.position+rball.radius*childaxis;
			radius=(upper-lower).norm()*static_cast<REAL>(0.5);
			position=(upper+lower)*static_cast<REAL>(0.5);
		}
	};
	struct ballnode
	{
		ball boundary;
		size_t leftdex;
		size_t rightdex;
		ballnode(const ball& b=ball()):
			boundary(b),
			leftdex(0),
			rightdex(0)
		{}
	};
private:
	static std::forward_list<ball> build_balltree_dfs(ball* bbegin,ball* bend);
public:
	std::vector<ballnode> allnodes;
	balltree(ball* bbegin,ball* bend)
	{
		std::forward_list<ball> ball_list=build_balltree_dfs(bbegin,bend); 
		allnodes=std::vector<ballnode>(ball_list.begin(),ball_list.end());
		for(size_t i=0;i<allnodes.size();i++)
		{
			if(allnodes[i].boundary.num_children > 0)
			{
				allnodes[i].leftdex=1;
				allnodes[i].rightdex=allnodes[i+1].boundary.num_children+2;
			}
		}
	}
	
};

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

template<class LeafType,unsigned int D,class REAL>
std::forward_list<typename balltree<LeafType,D,REAL>::ball> balltree<LeafType,D,REAL>::build_balltree_dfs(ball* bbegin,ball* bend)
{
	size_t n=bend-bbegin;
	if(n<=1)
	{
		std::forward_list<ball> lo;
		if(n==1)
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
	getDomEigenvector(childaxis,pos_lcov);
	
	std::for_each(bbegin,bend,[childaxis,pos_mean](ball& b)
	{
		b.temp_projector=(b.position-pos_mean).dot(childaxis);
	});
	ball* bmid=bbegin+n/2;
	std::nth_element(bbegin,bmid,bend,[](const ball& a,const ball& b)
	{
		return a.temp_projector < b.temp_projector;
	});
	auto left_list=build_balltree_dfs(bbegin,bmid);
	auto right_list=build_balltree_dfs(bmid,bend);
	
	ball newroot(left_list.front(),right_list.front());
	right_list.splice_after(right_list.before_begin(),left_list,left_list.before_begin(),left_list.end());
	right_list.push_front(newroot);
	return right_list;
}


#endif
