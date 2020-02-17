

namespace wp
{
inline bool Cutout::intersect(const Cutout& other,const balltransform2f& tform) const
{
	Eigen::Matrix<float,2,3> pm=tform.pmatrix();
	return tree.intersect_callable([&other,&tform,&pm](const ball2f& thisnode1)
	{
		return other.tree.intersect_callable([&tform,&thisnode1,&pm](const ball2f& thisnode2)
		{
			balltree2f::ball cpy=thisnode2;
			cpy.transform_in_place(tform);
			cpy.leaf.transform_in_place(pm);
			return thisnode1.intersect(cpy, wp::Triangle::intersect);
		});
	});
}

}
