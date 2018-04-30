namespace BulletSharp
{
	public class TransformUtil
	{
		public void CalculateDiffAxisAngle(btTransform^ transform0, btTransform^ transform1, btVector3^ axis, float^ angle)
		{
		}
		public void CalculateDiffAxisAngleQuaternion(btQuaternion^ orn0, btQuaternion^ orn1a, btVector3^ axis, float^ angle)
		{
		}
		public void CalculateVelocity(btTransform^ transform0, btTransform^ transform1, float timeStep, btVector3^ linVel, btVector3^ angVel)
		{
		}
		public void CalculateVelocityQuaternion(btVector3^ pos0, btVector3^ pos1, btQuaternion^ orn0, btQuaternion^ orn1, float timeStep, btVector3^ linVel, btVector3^ angVel)
		{
		}
		public void IntegrateTransform(btTransform^ curTrans, btVector3^ linvel, btVector3^ angvel, float timeStep, btTransform^ predictedTransform)
		{
		}
	}
	public class ConvexSeparatingDistanceUtil
	{
		public ConvexSeparatingDistanceUtil(float boundingRadiusA, float boundingRadiusB)
		{
		}
		public void GetConservativeSeparatingDistance()
		{
		}
		public void InitSeparatingDistance(btVector3^ separatingVector, float separatingDistance, btTransform^ transA, btTransform^ transB)
		{
		}
		public void UpdateSeparatingDistance(btTransform^ transA, btTransform^ transB)
		{
		}
	}
}
