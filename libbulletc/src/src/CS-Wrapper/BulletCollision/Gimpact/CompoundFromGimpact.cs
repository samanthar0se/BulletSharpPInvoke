namespace BulletSharp
{
	public class MyCallback
	{
		public MyCallback(int^ from, int^ to, int ignorePart, int ignoreTriangleIndex)
		{
		}
		public void ReportHit(int^ hitNormalLocal, int hitFraction, int partId, int triangleIndex)
		{
		}
	}
	public class MyInternalTriangleIndexCallback
	{
		public MyInternalTriangleIndexCallback(int^ colShape, btGImpactMeshShape^ meshShape, int depth)
		{
		}
		public void InternalProcessTriangleIndex(int^ triangle, int partId, int triangleIndex)
		{
		}
	}
}
