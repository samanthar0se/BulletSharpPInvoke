namespace BulletSharp
{
	public class BroadphaseAabbCallback
	{
		public void Process(btBroadphaseProxy^ proxy)
		{
		}
	}
	public class BroadphaseRayCallback
	{
		public BroadphaseRayCallback()
		{
		}
	}
	public class BroadphaseInterface
	{
		public void AabbTest(int^ aabbMin, int^ aabbMax, btBroadphaseAabbCallback^ callback)
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
	}
}
