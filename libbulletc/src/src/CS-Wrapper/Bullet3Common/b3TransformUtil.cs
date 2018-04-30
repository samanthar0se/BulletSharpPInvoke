namespace BulletSharp
{
	public class b3TransformUtil
	{
		public void CalculateDiffAxisAngle(b3Transform^ transform0, b3Transform^ transform1, b3Vector3^ axis, float^ angle)
		{
		}
		public void CalculateDiffAxisAngleQuaternion(b3Quaternion^ orn0, b3Quaternion^ orn1a, b3Vector3^ axis, float^ angle)
		{
		}
		public void CalculateVelocity(b3Transform^ transform0, b3Transform^ transform1, float timeStep, b3Vector3^ linVel, b3Vector3^ angVel)
		{
		}
		public void CalculateVelocityQuaternion(b3Vector3^ pos0, b3Vector3^ pos1, b3Quaternion^ orn0, b3Quaternion^ orn1, float timeStep, b3Vector3^ linVel, b3Vector3^ angVel)
		{
		}
		public void IntegrateTransform(b3Transform^ curTrans, b3Vector3^ linvel, b3Vector3^ angvel, float timeStep, b3Transform^ predictedTransform)
		{
		}
	}
	public class b3ConvexSeparatingDistanceUtil
	{
		public B3ConvexSeparatingDistanceUtil(float boundingRadiusA, float boundingRadiusB)
		{
		}
		public void GetConservativeSeparatingDistance()
		{
		}
		public void InitSeparatingDistance(b3Vector3^ separatingVector, float separatingDistance, b3Transform^ transA, b3Transform^ transB)
		{
		}
		public void UpdateSeparatingDistance(b3Transform^ transA, b3Transform^ transB)
		{
		}
	}
}
