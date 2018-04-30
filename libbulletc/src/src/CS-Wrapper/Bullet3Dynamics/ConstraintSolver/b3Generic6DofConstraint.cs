namespace BulletSharp
{
	public class b3RotationalLimitMotor
	{
		public B3RotationalLimitMotor()
		{
		}
		public B3RotationalLimitMotor(b3RotationalLimitMotor^ limot)
		{
		}
		public void IsLimited()
		{
		}
		public void NeedApplyTorques()
		{
		}
		public void SolveAngularLimits(int timeStep, int^ axis, int jacDiagABInv, b3RigidBodyData^ body0, b3RigidBodyData^ body1)
		{
		}
		public void TestLimitValue(int test_value)
		{
		}
	}
	public class b3TranslationalLimitMotor
	{
		public B3TranslationalLimitMotor()
		{
		}
		public B3TranslationalLimitMotor(b3TranslationalLimitMotor^ other)
		{
		}
		public void IsLimited(int limitIndex)
		{
		}
		public void NeedApplyForce(int limitIndex)
		{
		}
		public void SolveLinearAxis(int timeStep, int jacDiagABInv, b3RigidBodyData^ body1, int^ pointInA, b3RigidBodyData^ body2, int^ pointInB, int limit_index, int^ axis_normal_on_a, int^ anchorPos)
		{
		}
		public void TestLimitValue(int limitIndex, int test_value)
		{
		}
	}
}
