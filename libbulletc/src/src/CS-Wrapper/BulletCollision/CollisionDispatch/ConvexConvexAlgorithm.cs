namespace BulletSharp
{
	public class ConvexConvexAlgorithm
	{
		public ConvexConvexAlgorithm(int^ mf, btCollisionAlgorithmConstructionInfo^ ci, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap, btConvexPenetrationDepthSolver^ pdSolver, int numPerturbationIterations, int minimumPointsPerturbationThreshold)
		{
		}
		public void CalculateTimeOfImpact(btCollisionObject^ body0, btCollisionObject^ body1, int^ dispatchInfo, int^ resultOut)
		{
		}
		public void GetAllContactManifolds(int^ manifoldArray)
		{
		}
		public void GetManifold()
		{
		}
		public void ProcessCollision(btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap, int^ dispatchInfo, int^ resultOut)
		{
		}
		public void SetLowLevelOfDetail(bool useLowLevel)
		{
		}
	public class CreateFunc
	{
		public CreateFunc(btConvexPenetrationDepthSolver^ pdSolver)
		{
		}
		public void CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo^ ci, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap)
		{
		}
	}
	}
}
