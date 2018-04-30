namespace BulletSharp
{
	public class b3ParamsGridBroadphaseCL
	{
		public void GetMaxBodiesPerCell()
		{
		}
		public void SetMaxBodiesPerCell(int maxOverlap)
		{
		}
	}
	public class b3GpuGridBroadphase
	{
		public B3GpuGridBroadphase(int ctx, int device, int q)
		{
		}
		public void CalculateOverlappingPairs(int maxPairs)
		{
		}
		public void CalculateOverlappingPairsHost(int maxPairs)
		{
		}
		public void CreateFunc(int ctx, int device, int q)
		{
		}
		public void CreateLargeProxy(int^ aabbMin, int^ aabbMax, int userPtr, int collisionFilterGroup, int collisionFilterMask)
		{
		}
		public void CreateProxy(int^ aabbMin, int^ aabbMax, int userPtr, int collisionFilterGroup, int collisionFilterMask)
		{
		}
		public void GetAabbBufferWS()
		{
		}
		public void GetNumOverlap()
		{
		}
		public void GetOverlappingPairBuffer()
		{
		}
		public void WriteAabbsToGpu()
		{
		}
	}
}
