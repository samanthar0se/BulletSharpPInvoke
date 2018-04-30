namespace BulletSharp
{
	public class TriangleRaycastCallback
	{
		public TriangleRaycastCallback(int^ from, int^ to, uint flags)
		{
		}
		public void ProcessTriangle(int^ triangle, int partId, int triangleIndex)
		{
		}
		public void ReportHit(int^ hitNormalLocal, int hitFraction, int partId, int triangleIndex)
		{
		}
	[Flags]
	public enum EFlags
	{
		None = 0,
		FilterBackfaces = 1<<0,
		KeepUnflippedNormal = 1<<1,
		UseSubSimplexConvexCastRaytest = 1<<2,
		UseGjkConvexCastRaytest = 1<<3,
		Terminator = 0xFFFFFFFF
	}
	}
	public class TriangleConvexcastCallback
	{
		public TriangleConvexcastCallback(btConvexShape^ convexShape, int^ convexShapeFrom, int^ convexShapeTo, int^ triangleToWorld, int triangleCollisionMargin)
		{
		}
		public void ProcessTriangle(int^ triangle, int partId, int triangleIndex)
		{
		}
		public void ReportHit(int^ hitNormalLocal, int^ hitPointLocal, int hitFraction, int partId, int triangleIndex)
		{
		}
	}
}
