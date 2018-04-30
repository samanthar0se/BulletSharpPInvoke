namespace BulletSharp
{
	public class CollisionDispatcher
	{
		public CollisionDispatcher(btCollisionConfiguration^ collisionConfiguration)
		{
		}
		public void AllocateCollisionAlgorithm(int size)
		{
		}
		public void ClearManifold(int^ manifold)
		{
		}
		public void DefaultNearCallback(int^ collisionPair, btCollisionDispatcher^ dispatcher, int^ dispatchInfo)
		{
		}
		public void DispatchAllCollisionPairs(btOverlappingPairCache^ pairCache, int^ dispatchInfo, int^ dispatcher)
		{
		}
		public void FindAlgorithm(btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap, int^ sharedManifold, int queryType)
		{
		}
		public void FreeCollisionAlgorithm(void^ ptr)
		{
		}
		public void GetCollisionConfiguration()
		{
		}
		public void GetCollisionConfiguration()
		{
		}
		public void GetDispatcherFlags()
		{
		}
		public void GetInternalManifoldPointer()
		{
		}
		public void GetInternalManifoldPool()
		{
		}
		public void GetInternalManifoldPool()
		{
		}
		public void GetManifoldByIndexInternal(int index)
		{
		}
		public void GetManifoldByIndexInternal(int index)
		{
		}
		public void GetNearCallback()
		{
		}
		public void GetNewManifold(btCollisionObject^ b0, btCollisionObject^ b1)
		{
		}
		public void GetNumManifolds()
		{
		}
		public void NeedsCollision(btCollisionObject^ body0, btCollisionObject^ body1)
		{
		}
		public void NeedsResponse(btCollisionObject^ body0, btCollisionObject^ body1)
		{
		}
		public void RegisterClosestPointsCreateFunc(int proxyType0, int proxyType1, btCollisionAlgorithmCreateFunc^ createFunc)
		{
		}
		public void RegisterCollisionCreateFunc(int proxyType0, int proxyType1, btCollisionAlgorithmCreateFunc^ createFunc)
		{
		}
		public void ReleaseManifold(int^ manifold)
		{
		}
		public void SetCollisionConfiguration(btCollisionConfiguration^ config)
		{
		}
		public void SetDispatcherFlags(int flags)
		{
		}
		public void SetNearCallback(btNearCallback nearCallback)
		{
		}
	[Flags]
	public enum DispatcherFlags
	{
		None = 0,
		StaticStaticReported = 1,
		UseRelativeContactBreakingThreshold = 2,
		DisableContactpoolDynamicAllocation = 4
	}
	}
}
