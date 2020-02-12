#ifndef BALLTREE_HPP
#define BALLTREE_HPP

#include<Eigen/Dense>
#include<Eigen/Eigenvalues>
#include<algorithm>
#include<vector>
#include<forward_list>
#include<type_traits>

template<unsigned int D,class REAL=double>
struct balltransform
{
	Eigen::Matrix<REAL,D,D> rotation;
	Eigen::Matrix<REAL,D,1> offset;
	REAL scale;
	balltransform(const Eigen::Matrix<REAL,D,1>& tT=Eigen::Matrix<REAL,D,1>::Zero(),
				  const Eigen::Matrix<REAL,D,D>& tR=Eigen::Matrix<REAL,D,D>::Identity(),
				  const REAL& tS=1.0):
		rotation(tR),
		offset(tT),
		scale(tS)
	{}
	/*balltransform(const Eigen::Matrix<REAL,D,D+1>& a)
	{
		rotation=a.leftCols(D);
		offset=a.rightCols(1);
		scale=rotation.norm();
	}*/
};

template<unsigned int D,class REAL=double>
struct ballbase
{
	Eigen::Matrix<REAL,D,1> position;
	REAL radius;
	size_t num_children;
	REAL temp_projector;
	ballbase(const Eigen::Matrix<REAL,D,1>& pos=Eigen::Matrix<REAL,D,1>::Zero(),REAL trad=0.0);
	ballbase(const ballbase& lball,const ballbase& rball);
	bool intersect(const ballbase& b) const;
	void transform_in_place(const balltransform<D,REAL>& Rt);
};

template<class LeafType,unsigned int D,class REAL=double>
class balltree
{
public:
	struct ball: public ballbase<D,REAL>
	{
		LeafType leaf;
		ball();
		ball(const LeafType& lt,const Eigen::Matrix<REAL,D,1>& pos,REAL trad); //its illegal to make an empty leaf node
		ball(const ballbase<D,REAL>& a,const ballbase<D,REAL>& b,const LeafType& lt=LeafType());
		using ballbase<D,REAL>::intersect;
		
		template<class LeafBallType2,class LeafIntersectFunc>
		bool intersect(const LeafBallType2& other,LeafIntersectFunc leaf_intersect) const;
	};
private:
	template<class MakeParentFunc>
	static std::forward_list<ball> build_balltree_dfs(ball* bbegin,ball* bend,MakeParentFunc makeparent);
public:
	std::vector<ball> allnodes;
	
	balltree(ball* bbegin,ball* bend);
	
	template<class MakeParentFunc>
	balltree(ball* bbegin,ball* bend,MakeParentFunc makeparent);
	
	template<class ClientIntersectFunc>
	bool intersect_callable(ClientIntersectFunc client_intersect_func,size_t root=0) const;
	
	template<class LeafBallType,class LeafIntersectFunc>
	bool intersect(const LeafBallType& b,LeafIntersectFunc leaf_intersect,size_t root=0) const;
};


#include "BallTree.inl"

#endif
