namespace BulletSharp
{
	public class GjkPairDetector
	{
		public GjkPairDetector(btConvexShape^ objectA, btConvexShape^ objectB, int^ simplexSolver, btConvexPenetrationDepthSolver^ penetrationDepthSolver)
		{
		}
		public GjkPairDetector(btConvexShape^ objectA, btConvexShape^ objectB, int shapeTypeA, int shapeTypeB, int marginA, int marginB, int^ simplexSolver, btConvexPenetrationDepthSolver^ penetrationDepthSolver)
		{
		}
		public void GetCachedSeparatingAxis()
		{
		}
		public void GetCachedSeparatingDistance()
		{
		}
		public void GetClosestPoints(ClosestPointInput^ input, Result^ output, btIDebugDraw^ debugDraw, bool swapResults)
		{
		}
		public void GetClosestPointsNonVirtual(ClosestPointInput^ input, Result^ output, btIDebugDraw^ debugDraw)
		{
		}
		public void SetCachedSeperatingAxis(int^ seperatingAxis)
		{
		}
		public void SetIgnoreMargin(bool ignoreMargin)
		{
		}
		public void SetMinkowskiA(btConvexShape^ minkA)
		{
		}
		public void SetMinkowskiB(btConvexShape^ minkB)
		{
		}
		public void SetPenetrationDepthSolver(btConvexPenetrationDepthSolver^ penetrationDepthSolver)
		{
		}
	}
}
