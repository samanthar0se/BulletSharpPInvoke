namespace BulletSharp
{
	public class TriIndex
	{
		public TriIndex(int partId, int triangleIndex, btCollisionShape^ shape)
		{
		}
		public void GetPartId()
		{
		}
		public void GetTriangleIndex()
		{
		}
		public void GetUid()
		{
		}
	}
	public class SoftBodyTriangleCallback
	{
		public SoftBodyTriangleCallback(btDispatcher^ dispatcher, int^ body0Wrap, int^ body1Wrap, bool isSwapped)
		{
		}
		public void ClearCache()
		{
		}
		public void ProcessTriangle(int^ triangle, int partId, int triangleIndex)
		{
		}
		public void SetTimeStepAndCounters(int collisionMarginTriangle, int^ triObjWrap, int^ dispatchInfo, int^ resultOut)
		{
		}
	}
	public class SoftBodyConcaveCollisionAlgorithm
	{
		public SoftBodyConcaveCollisionAlgorithm(int^ ci, int^ body0Wrap, int^ body1Wrap, bool isSwapped)
		{
		}
		public void CalculateTimeOfImpact(int^ body0, int^ body1, int^ dispatchInfo, int^ resultOut)
		{
		}
		public void ClearCache()
		{
		}
		public void GetAllContactManifolds(int^ manifoldArray)
		{
		}
		public void ProcessCollision(int^ body0Wrap, int^ body1Wrap, int^ dispatchInfo, int^ resultOut)
		{
		}
	public class CreateFunc
	{
		public void CreateCollisionAlgorithm(int^ ci, int^ body0Wrap, int^ body1Wrap)
		{
		}
	}
	public class SwappedCreateFunc
	{
		public void CreateCollisionAlgorithm(int^ ci, int^ body0Wrap, int^ body1Wrap)
		{
		}
	}
	}
}
