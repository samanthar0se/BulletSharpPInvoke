namespace BulletSharp
{
	public class b3DbvtProxy
	{
		public B3DbvtProxy()
		{
		}
		public B3DbvtProxy(int^ aabbMin, int^ aabbMax, void^ userPtr, int collisionFilterGroup, int collisionFilterMask)
		{
		}
	}
	public class b3DynamicBvhBroadphase
	{
		public B3DynamicBvhBroadphase(int proxyCapacity, int^ paircache)
		{
		}
		public void AabbTest(int^ aabbMin, int^ aabbMax, b3BroadphaseAabbCallback^ callback)
		{
		}
		public void CalculateOverlappingPairs(int^ dispatcher)
		{
		}
		public void Collide(int^ dispatcher)
		{
		}
		public void CreateProxy(int^ aabbMin, int^ aabbMax, int objectIndex, void^ userPtr, int collisionFilterGroup, int collisionFilterMask)
		{
		}
		public void DestroyProxy(b3BroadphaseProxy^ proxy, int^ dispatcher)
		{
		}
		public void GetAabb(int objectId, int^ aabbMin, int^ aabbMax)
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
		public void GetVelocityPrediction()
		{
		}
		public void Optimize()
		{
		}
		public void PerformDeferredRemoval(int^ dispatcher)
		{
		}
		public void PrintStats()
		{
		}
		public void RayTest(int^ rayFrom, int^ rayTo, b3BroadphaseRayCallback^ rayCallback, int^ aabbMin, int^ aabbMax)
		{
		}
		public void ResetPool(int^ dispatcher)
		{
		}
		public void SetAabb(int objectId, int^ aabbMin, int^ aabbMax, int^ dispatcher)
		{
		}
		public void SetAabbForceUpdate(b3BroadphaseProxy^ absproxy, int^ aabbMin, int^ aabbMax, int^ )
		{
		}
		public void SetVelocityPrediction(int prediction)
		{
		}
	[Flags]
	public enum 
	{
		DynamicSet = 0,
		FixedSet = 1,
		Stagecount = 2
	}
	}
}
