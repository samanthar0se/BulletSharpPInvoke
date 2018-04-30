namespace BulletSharp
{
	public class GIM_QUANTIZED_BVH_NODE_ARRAY
	{
	}
	public class QuantizedBvhTree
	{
		public QuantizedBvhTree()
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
		public void Calc_quantization(GIM_BVH_DATA_ARRAY^ primitive_boxes, int boundMargin)
		{
		}
	}
	public class GImpactQuantizedBvh
	{
		public GImpactQuantizedBvh()
		{
		}
		public GImpactQuantizedBvh(btPrimitiveManagerBase^ primitive_manager)
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
