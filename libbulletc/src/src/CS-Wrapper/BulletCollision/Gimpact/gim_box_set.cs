namespace BulletSharp
{
	public class gim_pair_set
	{
		public Gim_pair_set()
		{
		}
		public void Push_pair(uint index1, uint index2)
		{
		}
		public void Push_pair_inv(uint index1, uint index2)
		{
		}
	}
	public class GIM_PRIMITIVE_MANAGER_PROTOTYPE
	{
		public void Get_primitive_box(uint prim_index, GIM_AABB^ primbox)
		{
		}
		public void Get_primitive_count()
		{
		}
		public void Get_primitive_triangle(uint prim_index, GIM_TRIANGLE^ triangle)
		{
		}
		public void Is_trimesh()
		{
		}
	}
	public class GIM_AABB_DATA
	{
	}
	public class GIM_BOX_TREE_NODE
	{
		public GIM_BOX_TREE_NODE()
		{
		}
	}
	public class GIM_BOX_TREE
	{
		public GIM_BOX_TREE()
		{
		}
		public void _build_sub_tree(gim_array^ primitive_boxes, uint startIndex, uint endIndex)
		{
		}
		public void _calc_splitting_axis(gim_array^ primitive_boxes, uint startIndex, uint endIndex)
		{
		}
		public void _sort_and_calc_splitting_index(gim_array^ primitive_boxes, uint startIndex, uint endIndex, uint splitAxis)
		{
		}
		public void Build_tree(gim_array^ primitive_boxes)
		{
		}
	}
	public class GIM_BOX_TREE_TEMPLATE_SET
	{
	}
	public class GIM_BOX_TREE_SET
	{
	}
	public class GIM_TREE_TREE_COLLIDER
	{
		public GIM_TREE_TREE_COLLIDER<BOX_SET_CLASS0, BOX_SET_CLASS1>()
		{
		}
	}
}
