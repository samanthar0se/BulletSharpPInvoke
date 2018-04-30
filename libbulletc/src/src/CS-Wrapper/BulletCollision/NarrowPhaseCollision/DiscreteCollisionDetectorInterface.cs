namespace BulletSharp
{
	public class DiscreteCollisionDetectorInterface
	{
		public void GetClosestPoints(ClosestPointInput^ input, Result^ output, btIDebugDraw^ debugDraw, bool swapResults)
		{
		}
	public class ClosestPointInput
	{
		public ClosestPointInput()
		{
		}
	}
	public class Result
	{
		public void AddContactPoint(int^ normalOnBInWorld, int^ pointInWorld, int depth)
		{
		}
		public void SetShapeIdentifiersA(int partId0, int index0)
		{
		}
		public void SetShapeIdentifiersB(int partId1, int index1)
		{
		}
	}
	}
	public class StorageResult
	{
		public StorageResult()
		{
		}
		public void AddContactPoint(int^ normalOnBInWorld, int^ pointInWorld, int depth)
		{
		}
	}
}
