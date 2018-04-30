namespace BulletSharp
{
	public class PairSet
	{
		public PairSet()
		{
		}
		public void Push_pair(int index1, int index2)
		{
		}
		public void Push_pair_inv(int index1, int index2)
		{
		}
	}
	public class GIM_BVH_DATA_ARRAY
	{
	}
	public class GIM_BVH_TREE_NODE_ARRAY
	{
	}
	public class BvhTree
	{
		public BvhTree()
		{
		}
		public void _build_sub_tree(GIM_BVH_DATA_ARRAY^ primitive_boxes, int startIndex, int endIndex)
		{
		}
		public void _calc_splitting_axis(GIM_BVH_DATA_ARRAY^ primitive_boxes, int startIndex, int endIndex)
		{
		}
		public void _sort_and_calc_splitting_index(GIM_BVH_DATA_ARRAY^ primitive_boxes, int startIndex, int endIndex, int splitAxis)
		{
		}
		public void Build_tree(GIM_BVH_DATA_ARRAY^ primitive_boxes)
		{
		}
	}
	public class PrimitiveManagerBase
	{
		public void Get_primitive_box(int prim_index, int^ primbox)
		{
		}
		public void Get_primitive_count()
		{
		}
		public void Get_primitive_triangle(int prim_index, btPrimitiveTriangle^ triangle)
		{
		}
		public void Is_trimesh()
		{
		}
	}
	public class GImpactBvh
	{
		public GImpactBvh()
		{
		}
		public GImpactBvh(btPrimitiveManagerBase^ primitive_manager)
		{
		}
		public void BoxQuery(int^ box, int )
		{
		}
		public void Refit()
		{
		}
	}
}
