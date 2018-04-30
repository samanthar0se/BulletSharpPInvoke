namespace BulletSharp
{
	public class CompoundCollisionAlgorithm
	{
		public CompoundCollisionAlgorithm(btCollisionAlgorithmConstructionInfo^ ci, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap, bool isSwapped)
		{
		}
		public void CalculateTimeOfImpact(btCollisionObject^ body0, btCollisionObject^ body1, int^ dispatchInfo, int^ resultOut)
		{
		}
		public void GetAllContactManifolds(int^ manifoldArray)
		{
		}
		public void GetChildAlgorithm(int n)
		{
		}
		public void PreallocateChildAlgorithms(btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap)
		{
		}
		public void ProcessCollision(btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap, int^ dispatchInfo, int^ resultOut)
		{
		}
		public void RemoveChildAlgorithms()
		{
		}
	public class CreateFunc
	{
		public void CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo^ ci, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap)
		{
		}
	}
	public class SwappedCreateFunc
	{
		public void CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo^ ci, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap)
		{
		}
	}
	}
}
