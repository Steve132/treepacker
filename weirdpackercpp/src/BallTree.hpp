#ifndef BALLTREE_HPP
#define BALLTREE_HPP


#include<Eigen/Dense>
#include<algorithm>
#include<vector>

template<class LeafType,unsigned int D,class REAL=double>
class balltree
{
public:
	struct ball
	{
		Eigen::Matrix<REAL,D,1> position;
		REAL radius;
		LeafType* leaf_id;
	};
	struct ballnode
	{
		ball boundary;
		ballnode left;
		ballnode right;
		ballnode(ball* bbegin,ball* bend);
	};
private:
	std::vector<ballnode> balls;
public:
	balltree(const ball* bbegin,const ball* bend)
	{
		std::vector<ball> balls(bbegin,bend);
	}
};

template<class LeafType,unsigned int D,class REAL=double>
balltree::ballnode(ball* bbegin,ball* bend)
{

	//sum (x-mx)*(y-my)' = sum x*y - mx * sum y - my * sum x - sum mx*my;
	//sum x*y - mx * sum y - my * sum x - sum mx*my;
	//sum x*y - 2*outer product of sums / N  + outer product of sums
	Eigen::Matrix<REAL,D,1> pos_sum;
	Eigen::Matrix<REAL,D,D> pos_lcov;
	size_t n=bend-bbegin;
	if(n==1)
	{
		boundary=*bbegin;
		return;
	}
	
	Eigen::Matrix<REAL,D,1> pos_mean,childaxis;
	
	if(n==2)
	{
		childaxis=bbegin[1].position-bbegin[0].position;
		pos_mean=bbegin[0].position+childaxis*0.5;
		childaxis.normalize();
		left=make_shared<balltree>(bbegin,bbegin+1);
		right=make_shared<balltree>(bbegin+1,bbegin+2);
	}
	else
	{
		for(const ball* bi=bbegin;bi != bend;bi++)
		{
			pos_sum+=bi->position;
			pos_lcov+=epos*epos.transposed();
		}
		REAL nf=static_cast<REAL>(n);
		pos_mean=pos_sum/nf;
		Eigen::Matrix<REAL,D,D> outer_prod_sums=pos_sum*pos_sum.transposed();
		Eigen::Matrix<REAL,D,D> pos_lcov=pos_lcov-outer_prod_sums*(2.0/nf+1.0/(nf*nf));
		childaxis=//evd(pos_lcov).largest_vector();
		
		//std::partition(bbegin,b  mean based splitting.
		
		std::nth_element(bbegin,bbegin+n/2,bend,
			[childaxis,pos_mean](const ball& a,const ball& b)
			{
				return (a.position-pos_mean).dot(childaxis) < (b.position-pos_mean).dot(childaxis);
			}
		);
		left=make_shared<balltree>(bbegin,bbegin+n/2);
		right=make_shared<balltree>(bbegin+n/2,bend);
	}
	boundary.leaf_id=nullptr;
	Eigen::Vector<REAL,D,1> lower=left->boundary.position-left->boundary.radius*childaxis;
	Eigen::Vector<REAL,D,1> upper=right->boundary.position+right->boundary.radius*childaxis;
	boundary.radius=(upper-lower).mag()/2.0;
	boundary.position=(upper+lower)/2.0;
}
