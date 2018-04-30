namespace BulletSharp
{
	public class ContinuousConvexCollision
	{
		public ContinuousConvexCollision(btConvexShape^ shapeA, btConvexShape^ shapeB, int^ simplexSolver, btConvexPenetrationDepthSolver^ penetrationDepthSolver)
		{
		}
		public ContinuousConvexCollision(btConvexShape^ shapeA, btStaticPlaneShape^ plane)
		{
		}
		public void CalcTimeOfImpact(int^ fromA, int^ toA, int^ fromB, int^ toB, CastResult^ result)
		{
		}
		public void ComputeClosestPoints(int^ transA, int^ transB, btPointCollector^ pointCollector)
		{
		}
	}
}
