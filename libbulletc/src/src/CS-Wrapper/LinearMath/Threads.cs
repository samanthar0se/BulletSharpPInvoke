namespace BulletSharp
{
	public class SpinMutex
	{
		public SpinMutex()
		{
		}
		public void Lock()
		{
		}
		public void TryLock()
		{
		}
		public void Unlock()
		{
		}
	}
	public class IParallelForBody
	{
		public void ForLoop(int iBegin, int iEnd)
		{
		}
	}
	public class ITaskScheduler
	{
		public ITaskScheduler(char^ name)
		{
		}
		public void Activate()
		{
		}
		public void Deactivate()
		{
		}
		public void GetMaxNumThreads()
		{
		}
		public void GetName()
		{
		}
		public void GetNumThreads()
		{
		}
		public void ParallelFor(int iBegin, int iEnd, int grainSize, btIParallelForBody^ body)
		{
		}
		public void SetNumThreads(int numThreads)
		{
		}
	}
}
