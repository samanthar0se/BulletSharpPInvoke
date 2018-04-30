namespace BulletSharp
{
	public class b3SortData
	{
	}
	public class b3RadixSort32CL
	{
		public B3RadixSort32CL(int ctx, int device, int queue, int initialCapacity)
		{
		}
		public void Execute(b3OpenCLArray^ keysIn, b3OpenCLArray^ keysOut, b3OpenCLArray^ valuesIn, b3OpenCLArray^ valuesOut, int n, int sortBits)
		{
		}
		public void Execute(b3OpenCLArray^ keysInOut, int sortBits)
		{
		}
		public void Execute(b3OpenCLArray^ keyValuesInOut, int sortBits)
		{
		}
		public void ExecuteHost(b3OpenCLArray^ keyValuesInOut, int sortBits)
		{
		}
		public void ExecuteHost(int )
		{
		}
	public class b3ConstData
	{
	}
	public enum 
	{
		DataAlignment = 256,
		WgSize = 64,
		BlockSize = 256,
		ElementsPerWorkItem = (BlockSize/WgSize),
		BitsPerPass = 4,
		NumBucket = (1<<BitsPerPass),
		NumWgs = 20*6
	}
	}
}
