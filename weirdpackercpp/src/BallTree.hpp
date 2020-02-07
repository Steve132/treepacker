#ifndef BALLTREE_HPP
#define BALLTREE_HPP

#include<Eigen/Dense>
#include<Eigen/Eigenvalues>
#include<algorithm>
#include<vector>
#include<forward_list>
#include<type_traits>

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
	static std::forward_list<ball> build_balltree_dfs(ball* bbegin,ball* bend);
public:
	std::vector<ball> allnodes;
	
	balltree(ball* bbegin,ball* bend);
	
	template<class ClientIntersectFunc>
	bool intersect_callable(ClientIntersectFunc client_intersect_func,size_t root=0) const;
	
	template<class LeafBallType,class LeafIntersectFunc>
	bool intersect(const LeafBallType& b,LeafIntersectFunc leaf_intersect,size_t root=0) const;
};


#include "BallTree.inl"

#endif
