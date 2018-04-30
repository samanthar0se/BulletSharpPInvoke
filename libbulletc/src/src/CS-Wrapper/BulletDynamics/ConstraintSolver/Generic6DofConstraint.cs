namespace BulletSharp
{
	public class RotationalLimitMotor
	{
		public RotationalLimitMotor()
		{
		}
		public RotationalLimitMotor(btRotationalLimitMotor^ limot)
		{
		}
		public void IsLimited()
		{
		}
		public void NeedApplyTorques()
		{
		}
		public void SolveAngularLimits(int timeStep, int^ axis, int jacDiagABInv, btRigidBody^ body0, btRigidBody^ body1)
		{
		}
		public void TestLimitValue(int test_value)
		{
		}
	}
	public class TranslationalLimitMotor
	{
		public TranslationalLimitMotor()
		{
		}
		public TranslationalLimitMotor(btTranslationalLimitMotor^ other)
		{
		}
		public void IsLimited(int limitIndex)
		{
		}
		public void NeedApplyForce(int limitIndex)
		{
		}
		public void SolveLinearAxis(int timeStep, int jacDiagABInv, btRigidBody^ body1, int^ pointInA, btRigidBody^ body2, int^ pointInB, int limit_index, int^ axis_normal_on_a, int^ anchorPos)
		{
		}
		public void TestLimitValue(int limitIndex, int test_value)
		{
		}
	}
	public class Generic6DofConstraintData
	{
	}
	public class Generic6DofConstraintDoubleData2
	{
	}
}
