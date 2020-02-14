

namespace wp
{
inline bool Cutout::intersect(const Cutout& other,const balltransform2f& tform) const
{
	return tree.intersect_callable([&other,&tform](const ball2f& thisnode1)
	{
		return other.tree.intersect_callable([&tform,&thisnode1](const ball2f& thisnode2)
		{
			balltree2f::ball cpy=thisnode2;
			return thisnode1.intersect(cpy, wp::Triangle::intersect);
		});
	});
}

}
