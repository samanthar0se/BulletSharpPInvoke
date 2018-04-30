namespace BulletSharp
{
	public class DispatcherInfo
	{
		public DispatcherInfo()
		{
		}
	public enum DispatchFunc
	{
		Discrete = 1,
		Continuous
	}
	}
	public class Dispatcher
	{
		public void AllocateCollisionAlgorithm(int size)
		{
		}
		public void ClearManifold(btPersistentManifold^ manifold)
		{
		}
		public void DispatchAllCollisionPairs(btOverlappingPairCache^ pairCache, btDispatcherInfo^ dispatchInfo, btDispatcher^ dispatcher)
		{
		}
		public void FindAlgorithm(btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap, btPersistentManifold^ sharedManifold, ebtDispatcherQueryType queryType)
		{
		}
		public void FreeCollisionAlgorithm(void^ ptr)
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
		public void ReleaseManifold(btPersistentManifold^ manifold)
		{
		}
	}
}
