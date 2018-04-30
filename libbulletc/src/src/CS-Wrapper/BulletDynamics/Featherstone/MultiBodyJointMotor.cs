namespace BulletSharp
{
	public class MultiBodyJointMotor
	{
		public MultiBodyJointMotor(btMultiBody^ body, int link, int desiredVelocity, int maxMotorImpulse)
		{
		}
		public MultiBodyJointMotor(btMultiBody^ body, int link, int linkDoF, int desiredVelocity, int maxMotorImpulse)
		{
		}
		public void CreateConstraintRows(int^ constraintRows, btMultiBodyJacobianData^ data, int^ infoGlobal)
		{
		}
		public void DebugDraw(btIDebugDraw^ drawer)
		{
		}
		public void FinalizeMultiDof()
		{
		}
		public void GetErp()
		{
		}
		public void GetIslandIdA()
		{
		}
		public void GetIslandIdB()
		{
		}
		public void SetErp(int erp)
		{
		}
		public void SetPositionTarget(int posTarget, int kp)
		{
		}
		public void SetRhsClamp(int rhsClamp)
		{
		}
		public void SetVelocityTarget(int velTarget, int kd)
		{
		}
	}
}
