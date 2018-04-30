namespace BulletSharp
{
	public class GImpactCollisionAlgorithm
	{
		public GImpactCollisionAlgorithm(int^ ci, int^ body0Wrap, int^ body1Wrap)
		{
		}
		public void CalculateTimeOfImpact(int^ body0, int^ body1, int^ dispatchInfo, int^ resultOut)
		{
		}
		public void Collide_gjk_triangles(int^ body0Wrap, int^ body1Wrap, btGImpactMeshShapePart^ shape0, btGImpactMeshShapePart^ shape1, int^ pairs, int pair_count)
		{
		}
		public void Collide_sat_triangles(int^ body0Wrap, int^ body1Wrap, btGImpactMeshShapePart^ shape0, btGImpactMeshShapePart^ shape1, int^ pairs, int pair_count)
		{
		}
		public void Convex_vs_convex_collision(int^ body0Wrap, int^ body1Wrap, int^ shape0, int^ shape1)
		{
		}
		public void GetAllContactManifolds(int^ manifoldArray)
		{
		}
		public void GetFace0()
		{
		}
		public void GetFace1()
		{
		}
		public void GetPart0()
		{
		}
		public void GetPart1()
		{
		}
		public void Gimpact_vs_compoundshape(int^ body0Wrap, int^ body1Wrap, btGImpactShapeInterface^ shape0, int^ shape1, bool swapped)
		{
		}
		public void Gimpact_vs_concave(int^ body0Wrap, int^ body1Wrap, btGImpactShapeInterface^ shape0, int^ shape1, bool swapped)
		{
		}
		public void Gimpact_vs_gimpact(int^ body0Wrap, int^ body1Wrap, btGImpactShapeInterface^ shape0, btGImpactShapeInterface^ shape1)
		{
		}
		public void Gimpact_vs_gimpact_find_pairs(int^ trans0, int^ trans1, btGImpactShapeInterface^ shape0, btGImpactShapeInterface^ shape1, btPairSet^ pairset)
		{
		}
		public void Gimpact_vs_shape(int^ body0Wrap, int^ body1Wrap, btGImpactShapeInterface^ shape0, int^ shape1, bool swapped)
		{
		}
		public void Gimpact_vs_shape_find_pairs(int^ trans0, int^ trans1, btGImpactShapeInterface^ shape0, int^ shape1, int )
		{
		}
		public void Gimpacttrimeshpart_vs_plane_collision(int^ body0Wrap, int^ body1Wrap, btGImpactMeshShapePart^ shape0, int^ shape1, bool swapped)
		{
		}
		public void InternalGetResultOut()
		{
		}
		public void ProcessCollision(int^ body0Wrap, int^ body1Wrap, int^ dispatchInfo, int^ resultOut)
		{
		}
		public void RegisterAlgorithm(int^ dispatcher)
		{
		}
		public void SetFace0(int value)
		{
		}
		public void SetFace1(int value)
		{
		}
		public void SetPart0(int value)
		{
		}
		public void SetPart1(int value)
		{
		}
		public void Shape_vs_shape_collision(int^ body0, int^ body1, int^ shape0, int^ shape1)
		{
		}
	public class CreateFunc
	{
		public void CreateCollisionAlgorithm(int^ ci, int^ body0Wrap, int^ body1Wrap)
		{
		}
	}
	}
}
