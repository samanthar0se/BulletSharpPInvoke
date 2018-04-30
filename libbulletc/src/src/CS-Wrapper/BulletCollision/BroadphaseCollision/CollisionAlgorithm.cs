namespace BulletSharp
{
	public class CollisionAlgorithmConstructionInfo
	{
		public CollisionAlgorithmConstructionInfo()
		{
		}
		public CollisionAlgorithmConstructionInfo(btDispatcher^ dispatcher, int temp)
		{
		}
	}
	public class CollisionAlgorithm
	{
		public CollisionAlgorithm()
		{
		}
		public CollisionAlgorithm(btCollisionAlgorithmConstructionInfo^ ci)
		{
		}
		public void CalculateTimeOfImpact(btCollisionObject^ body0, btCollisionObject^ body1, btDispatcherInfo^ dispatchInfo, btManifoldResult^ resultOut)
		{
		}
		public void GetAllContactManifolds(int^ manifoldArray)
		{
		}
		public void ProcessCollision(btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap, btDispatcherInfo^ dispatchInfo, btManifoldResult^ resultOut)
		{
		}
	}
}
