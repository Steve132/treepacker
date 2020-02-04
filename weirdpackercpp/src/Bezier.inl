#include<cmath>

namespace
{
//paul bourkes code http://paulbourke.net/geometry/bezier/index.html
template<class XYZ>
inline XYZ BezierInterp(XYZ *p,int n,double mu)
{
   int k,kn,nn,nkn;
   double blend,muk,munk;
   XYZ b = {0.0,0.0,0.0};

   muk = 1;
   munk = std::pow(1-mu,(double)n);

   for (k=0;k<=n;k++) {
      nn = n;
      kn = k;
      nkn = n - k;
      blend = muk * munk;
      muk *= mu;
      munk /= (1-mu);
      while (nn >= 1) {
         blend *= nn;
         nn--;
         if (kn > 1) {
            blend /= (double)kn;
            kn--;
         }
         if (nkn > 1) {
            blend /= (double)nkn;
            nkn--;
         }
      }
	  b += p[k]*blend;
   }

   return(b);
}

template<class XYZ>
inline XYZ Bezier2Interp(XYZ p1,XYZ p2,XYZ p3,double mu)
{
   double mum1,mum12,mu2;
   XYZ p;

   mu2 = mu * mu;
   mum1 = 1 - mu;
   mum12 = mum1 * mum1;
   p = p1 * mum12 + p2 * (2.0*mum1 * mu) + p3 * mu2;
   
   return(p);
}

/*
   Four control point Bezier interpolation
   mu ranges from 0 to 1, start to end of curve
*/
template<class XYZ>
inline XYZ Bezier3Interp(XYZ p1,XYZ p2,XYZ p3,XYZ p4,double mu)
{
   double mum1,mum13,mu3;
   XYZ p;

   mum1 = 1 - mu;
   mum13 = mum1 * mum1 * mum1;
   mu3 = mu * mu * mu;

   p = p1*mum13 + p2*(3.0*mu*mum1*mum1) + p3*(3.0*mu*mu*mum1) + p4*(mu3);

   return(p);
}

template<size_t N,class VectorType>
struct interp_implementation
{
	static inline VectorType doit(const std::array<VectorType,N+1>& cp,double mu)
	{
		return BezierInterp(&cp[0],N,mu);
	}
};

template<class VectorType>
struct interp_implementation<2,VectorType>
{
	static inline VectorType doit(const std::array<VectorType,2+1>& cp,double mu)
	{
		return Bezier2Interp(cp[0],cp[1],cp[2],mu);
	}
};
template<class VectorType>
struct interp_implementation<3,VectorType>
{
	static inline VectorType doit(const std::array<VectorType,3+1>& cp,double mu)
	{
		return Bezier3Interp(cp[0],cp[1],cp[2],cp[3],mu);
	}
};
}

template<size_t N,class VectorType>
inline VectorType Bezier<N,VectorType>::interpolate(double mu) const
{
	return interp_implementation<N,VectorType>::doit(controlpoints,mu);
}
template<size_t N,class VectorType>
inline double Bezier<N,VectorType>::average_segment_norm(size_t n) const
{
	double tot=0.0;
	VectorType v0=interpolate(0.0);
	double d=((double)1.0)/((double)n);
	for(size_t i=1;i<n;i++)
	{
		VectorType v1=interpolate(((double)i)*d);
		tot+=(v1-v0).norm();
	}
	return tot*d;
}
template<size_t N,class VectorType>
inline size_t Bezier<N,VectorType>::estimate_num_segments(double norm_target,size_t start) const
{
	double an=average_segment_norm(start);
	if(an < norm_target)
	{
		while(an < norm_target)
		{
			start/=2;
			an=average_segment_norm(start);
		}
	}
	else
	{
		while(an >= norm_target)
		{
			start*=2;
			an=average_segment_norm(start);
		}
	}
	return start;
}

