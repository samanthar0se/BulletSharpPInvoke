namespace BulletSharp
{
	public class DbvtProxy
	{
		public DbvtProxy(int^ aabbMin, int^ aabbMax, void^ userPtr, int collisionFilterGroup, int collisionFilterMask)
		{
		}
	}
	public class DbvtBroadphase
	{
		public DbvtBroadphase(int^ paircache)
		{
		}
		public void AabbTest(int^ aabbMin, int^ aabbMax, int^ callback)
		{
		}
		public void Benchmark(int^ )
		{
		}
		public void CalculateOverlappingPairs(int^ dispatcher)
		{
		}
		public void Collide(int^ dispatcher)
		{
		}
		public void CreateProxy(int^ aabbMin, int^ aabbMax, int shapeType, void^ userPtr, int collisionFilterGroup, int collisionFilterMask, int^ dispatcher)
		{
		}
		public void DestroyProxy(int^ proxy, int^ dispatcher)
		{
		}
		public void GetAabb(int^ proxy, int^ aabbMin, int^ aabbMax)
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
		public void RayTest(int^ rayFrom, int^ rayTo, int^ rayCallback, int^ aabbMin, int^ aabbMax)
		{
		}
		public void ResetPool(int^ dispatcher)
		{
		}
		public void SetAabb(int^ proxy, int^ aabbMin, int^ aabbMax, int^ dispatcher)
		{
		}
		public void SetAabbForceUpdate(int^ absproxy, int^ aabbMin, int^ aabbMax, int^ )
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
