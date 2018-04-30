namespace BulletSharp
{
	public class SphereBoxCollisionAlgorithm
	{
		public SphereBoxCollisionAlgorithm(btPersistentManifold^ mf, btCollisionAlgorithmConstructionInfo^ ci, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap, bool isSwapped)
		{
		}
		public void CalculateTimeOfImpact(btCollisionObject^ body0, btCollisionObject^ body1, int^ dispatchInfo, int^ resultOut)
		{
		}
		public void GetAllContactManifolds(int^ manifoldArray)
		{
		}
		public void GetSphereDistance(btCollisionObjectWrapper^ boxObjWrap, int^ v3PointOnBox, int^ normal, int^ penetrationDepth, int^ v3SphereCenter, int fRadius, int maxContactDistance)
		{
		}
		public void GetSpherePenetration(int )
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
