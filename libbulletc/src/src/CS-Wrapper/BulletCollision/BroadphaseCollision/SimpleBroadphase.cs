namespace BulletSharp
{
	public class SimpleBroadphaseProxy
	{
		public SimpleBroadphaseProxy()
		{
		}
		public SimpleBroadphaseProxy(int^ minpt, int^ maxpt, int shapeType, void^ userPtr, int collisionFilterGroup, int collisionFilterMask)
		{
		}
	}
	public class SimpleBroadphase
	{
		public SimpleBroadphase(int maxProxies, btOverlappingPairCache^ overlappingPairCache)
		{
		}
		public void AabbOverlap(btSimpleBroadphaseProxy^ proxy0, btSimpleBroadphaseProxy^ proxy1)
		{
		}
		public void AabbTest(int^ aabbMin, int^ aabbMax, btBroadphaseAabbCallback^ callback)
		{
		}
		public void AllocHandle()
		{
		}
		public void CalculateOverlappingPairs(btDispatcher^ dispatcher)
		{
		}
		public void CreateProxy(int^ aabbMin, int^ aabbMax, int shapeType, void^ userPtr, int collisionFilterGroup, int collisionFilterMask, btDispatcher^ dispatcher)
		{
		}
		public void DestroyProxy(btBroadphaseProxy^ proxy, btDispatcher^ dispatcher)
		{
		}
		public void FreeHandle(btSimpleBroadphaseProxy^ proxy)
		{
		}
		public void GetAabb(btBroadphaseProxy^ proxy, int^ aabbMin, int^ aabbMax)
		{
		}
		public void GetBroadphaseAabb(int^ aabbMin, int^ aabbMax)
		{
		}
		public void GetOverlappingPairCache()
		{
		}
		public void GetOverlappingPairCache()
		{
		}
		public void GetSimpleProxyFromProxy(btBroadphaseProxy^ proxy)
		{
		}
		public void GetSimpleProxyFromProxy(btBroadphaseProxy^ proxy)
		{
		}
		public void PrintStats()
		{
		}
		public void RayTest(int^ rayFrom, int^ rayTo, btBroadphaseRayCallback^ rayCallback, int^ aabbMin, int^ aabbMax)
		{
		}
		public void ResetPool(btDispatcher^ dispatcher)
		{
		}
		public void SetAabb(btBroadphaseProxy^ proxy, int^ aabbMin, int^ aabbMax, btDispatcher^ dispatcher)
		{
		}
		public void TestAabbOverlap(btBroadphaseProxy^ proxy0, btBroadphaseProxy^ proxy1)
		{
		}
		public void Validate()
		{
		}
	}
}
