namespace BulletSharp
{
	public class AxisSweep3Internal
	{
		public AxisSweep3Internal<BP_FP_INT_TYPE>(int^ worldAabbMin, int^ worldAabbMax, [unexposed type] handleMask, [unexposed type] handleSentinel, [unexposed type] maxHandles, btOverlappingPairCache^ pairCache, bool disableRaycastAccelerator)
		{
		}
		public void AabbTest(int^ aabbMin, int^ aabbMax, btBroadphaseAabbCallback^ callback)
		{
		}
		public void AddHandle(int^ aabbMin, int^ aabbMax, void^ pOwner, int collisionFilterGroup, int collisionFilterMask, btDispatcher^ dispatcher)
		{
		}
		public void AllocHandle()
		{
		}
		public void BT_DECLARE_ALIGNED_ALLOCATOR()
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
		public void FreeHandle([unexposed type] handle)
		{
		}
		public void GetAabb(btBroadphaseProxy^ proxy, int^ aabbMin, int^ aabbMax)
		{
		}
		public void GetBroadphaseAabb(int^ aabbMin, int^ aabbMax)
		{
		}
		public void GetNumHandles()
		{
		}
		public void GetOverlappingPairCache()
		{
		}
		public void GetOverlappingPairCache()
		{
		}
		public void GetOverlappingPairUserCallback()
		{
		}
		public void PrintStats()
		{
		}
		public void ProcessAllOverlappingPairs(btOverlapCallback^ callback)
		{
		}
		public void Quantize([unexposed type]^ out, int^ point, int isMax)
		{
		}
		public void RayTest(int^ rayFrom, int^ rayTo, btBroadphaseRayCallback^ rayCallback, int^ aabbMin, int^ aabbMax)
		{
		}
		public void RemoveHandle([unexposed type] handle, btDispatcher^ dispatcher)
		{
		}
		public void SetAabb(btBroadphaseProxy^ proxy, int^ aabbMin, int^ aabbMax, btDispatcher^ dispatcher)
		{
		}
		public void SetOverlappingPairUserCallback(btOverlappingPairCallback^ pairCallback)
		{
		}
		public void SortMaxDown(int axis, [unexposed type] edge, btDispatcher^ dispatcher, bool updateOverlaps)
		{
		}
		public void SortMaxUp(int axis, [unexposed type] edge, btDispatcher^ dispatcher, bool updateOverlaps)
		{
		}
		public void SortMinDown(int axis, [unexposed type] edge, btDispatcher^ dispatcher, bool updateOverlaps)
		{
		}
		public void SortMinUp(int axis, [unexposed type] edge, btDispatcher^ dispatcher, bool updateOverlaps)
		{
		}
		public void TestAabbOverlap(btBroadphaseProxy^ proxy0, btBroadphaseProxy^ proxy1)
		{
		}
		public void TestOverlap2D(Handle^ pHandleA, Handle^ pHandleB, int axis0, int axis1)
		{
		}
		public void UnQuantize(btBroadphaseProxy^ proxy, int^ aabbMin, int^ aabbMax)
		{
		}
		public void UpdateHandle([unexposed type] handle, int^ aabbMin, int^ aabbMax, btDispatcher^ dispatcher)
		{
		}
	public class Edge
	{
		public void IsMax()
		{
		}
	}
	public class Handle
	{
		public void BT_DECLARE_ALIGNED_ALLOCATOR()
		{
		}
	}
	}
}
