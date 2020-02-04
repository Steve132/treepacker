


/*static double average_distance3(XY p1,XY p2,XY p3,size_t n)
{
	double totaldist=0.0;
	double tdelta=1.0/((double)n);
	for(size_t i=0;i<n;i++)
	{
		double t0=i;
		totaldist+=(Bezier3(p1,p2,p3,t0*tdelta)-Bezier3(p1,p2,p3,(t0+1.0)*tdelta)).norm();
	}
	return totaldist*tdelta;
}

static size_t estimate_averagedistance3(XY p1,XY p2,XY p3,size_t n)
*/
