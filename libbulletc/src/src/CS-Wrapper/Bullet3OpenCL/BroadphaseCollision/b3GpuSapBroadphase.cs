namespace BulletSharp
{
	public class b3GpuSapBroadphase
	{
		public B3GpuSapBroadphase(int ctx, int device, int q, b3GpuSapKernelType kernelType)
		{
		}
		public void CalculateOverlappingPairs(int maxPairs)
		{
		}
		public void CalculateOverlappingPairsHost(int maxPairs)
		{
		}
		public void CalculateOverlappingPairsHostIncremental3Sap()
		{
		}
		public void CreateFuncBarrier(int ctx, int device, int q)
		{
		}
		public void CreateFuncBruteForceCpu(int ctx, int device, int q)
		{
		}
		public void CreateFuncBruteForceGpu(int ctx, int device, int q)
		{
		}
		public void CreateFuncLocalMemory(int ctx, int device, int q)
		{
		}
		public void CreateFuncOriginal(int ctx, int device, int q)
		{
		}
		public void CreateLargeProxy(b3Vector3^ aabbMin, b3Vector3^ aabbMax, int userPtr, int collisionFilterGroup, int collisionFilterMask)
		{
		}
		public void CreateProxy(b3Vector3^ aabbMin, b3Vector3^ aabbMax, int userPtr, int collisionFilterGroup, int collisionFilterMask)
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
		public void Init3dSap()
		{
		}
		public void Reset()
		{
		}
		public void WriteAabbsToGpu()
		{
		}
	public enum b3GpuSapKernelType
	{
		BruteForceCpu = 1,
		BruteForceGpu,
		Original,
		Barrier,
		LocalSharedMemory
	}
	}
}
