namespace BulletSharp
{
	public class EmptyAlgorithm
	{
		public EmptyAlgorithm(btCollisionAlgorithmConstructionInfo^ ci)
		{
		}
		public void CalculateTimeOfImpact(btCollisionObject^ body0, btCollisionObject^ body1, int^ dispatchInfo, int^ resultOut)
		{
		}
		public void GetAllContactManifolds(int^ manifoldArray)
		{
		}
		public void ProcessCollision(btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap, int^ dispatchInfo, int^ resultOut)
		{
		}
	public class CreateFunc
	{
		public void CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo^ ci, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap)
		{
		}
	}
	}
}
