namespace BulletSharp
{
	public class b3LauncherCL
	{
		public B3LauncherCL(int queue, int kernel, char^ name)
		{
		}
		public void DeserializeArgs(byte^ buf, int bufSize, int ctx)
		{
		}
		public void EnableSerialization(bool serialize)
		{
		}
		public void GetArgument(int index)
		{
		}
		public void GetNumArguments()
		{
		}
		public void GetSerializationBufferSize()
		{
		}
		public void Launch1D(int numThreads, int localSize)
		{
		}
		public void Launch2D(int numThreadsX, int numThreadsY, int localSizeX, int localSizeY)
		{
		}
		public void SerializeArguments(byte^ destBuffer, int destBufferCapacity)
		{
		}
		public void SerializeToFile(char^ fileName, int numWorkItems)
		{
		}
		public void SetBuffer(int clBuffer)
		{
		}
		public void SetBuffers(b3BufferInfoCL^ buffInfo, int n)
		{
		}
		public void ValidateResults(byte^ goldBuffer, int goldBufferCapacity, int ctx)
		{
		}
	}
}
