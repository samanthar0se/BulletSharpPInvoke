namespace BulletSharp
{
	public class ConvexPlaneCollisionAlgorithm
	{
		public ConvexPlaneCollisionAlgorithm(btPersistentManifold^ mf, btCollisionAlgorithmConstructionInfo^ ci, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap, bool isSwapped, int numPerturbationIterations, int minimumPointsPerturbationThreshold)
		{
		}
		public void CalculateTimeOfImpact(btCollisionObject^ body0, btCollisionObject^ body1, int^ dispatchInfo, int^ resultOut)
		{
		}
		public void CollideSingleContact(int^ perturbeRot, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap, int^ dispatchInfo, int^ resultOut)
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
		public CreateFunc()
		{
		}
		public void CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo^ ci, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap)
		{
		}
	}
	}
}
