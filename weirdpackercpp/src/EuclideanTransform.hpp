#ifndef EUCLIDEAN_TRANSFORM_HPP
#define EUCLIDEAN_TRANSFORM_HPP

#include<Eigen/Dense>

template<unsigned int D,class REAL=double>
struct EuclideanTransform
{
	Eigen::Matrix<REAL,D,1> translation;
	Eigen::Matrix<REAL,D,D> rotation;
	REAL scale;
	EuclideanTransform(const Eigen::Matrix<REAL,D,1>& tT=Eigen::Matrix<REAL,D,1>::Zero(),
				  const Eigen::Matrix<REAL,D,D>& tR=Eigen::Matrix<REAL,D,D>::Identity(),
				  const REAL& tS=1.0);
	
	static EuclideanTransform from_euclidean_pmatrix(const Eigen::Matrix<REAL,D,D+1>& euclidean_matrix);
	
	Eigen::Matrix<REAL,D,D+1> pmatrix() const;
	EuclideanTransform& operator*=(const EuclideanTransform& other);
	EuclideanTransform operator*(const EuclideanTransform& other) const;
	Eigen::Matrix<REAL,D,1> operator*(const Eigen::Matrix<REAL,D,1>& ov) const;
};



template<unsigned int D,class REAL>
inline EuclideanTransform<D,REAL>::EuclideanTransform(const Eigen::Matrix<REAL,D,1>& tT,
			  const Eigen::Matrix<REAL,D,D>& tR,
			  const REAL& tS):
	translation(tT),
	rotation(tR),
	scale(tS)
{}
template<unsigned int D,class REAL>
inline Eigen::Matrix<REAL,D,D+1> EuclideanTransform<D,REAL>::pmatrix() const
{
	Eigen::Matrix<REAL,D,D+1> tform;
	tform.leftCols(D)=rotation*scale;
	tform.rightCols(1)=translation;
	return tform;
}
template<unsigned int D,class REAL>
inline 	EuclideanTransform<D,REAL> EuclideanTransform<D,REAL>::operator*(const EuclideanTransform& other) const
{
	return EuclideanTransform(translation+rotation*(other.translation*scale),
		rotation*other.rotation,
		scale*other.scale
	);
}
template<unsigned int D,class REAL>
inline EuclideanTransform<D,REAL>& EuclideanTransform<D,REAL>::operator*=(const EuclideanTransform& other)
{
	//[I | T1]R1*[I | T2]R2
	//TForm result(Eigen::Vector3d(position + (rotation * b.position)), Eigen::Quaterniond(rotation * b.rotation));
	translation+=rotation*(translation.position*scale);
	rotation*=other.rotation;
	scale*=other.scale;
	return *this;
}
template<unsigned int D,class REAL>
inline EuclideanTransform<D,REAL> EuclideanTransform<D,REAL>::from_euclidean_pmatrix(const Eigen::Matrix<REAL,D,D+1>& a)
{
	//[RS | TS] 
	EuclideanTransform trs(
		a.rightCols(1),
		a.leftCols(D));
	trs.scale=std::sqrt(trs.rotation.squaredNorm()/static_cast<REAL>(D));
	trs.rotation/=trs.scale;
	trs.translation/=trs.scale;
}

#endif
