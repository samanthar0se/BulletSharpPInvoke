namespace BulletSharp
{
	public class MultiBodySliderConstraint
	{
		public MultiBodySliderConstraint(btMultiBody^ body, int link, int^ bodyB, int^ pivotInA, int^ pivotInB, int^ frameInA, int^ frameInB, int^ jointAxis)
		{
		}
		public MultiBodySliderConstraint(btMultiBody^ bodyA, int linkA, btMultiBody^ bodyB, int linkB, int^ pivotInA, int^ pivotInB, int^ frameInA, int^ frameInB, int^ jointAxis)
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
		public void GetFrameInA()
		{
		}
		public void GetFrameInB()
		{
		}
		public void GetIslandIdA()
		{
		}
		public void GetIslandIdB()
		{
		}
		public void GetJointAxis()
		{
		}
		public void GetPivotInA()
		{
		}
		public void GetPivotInB()
		{
		}
		public void SetFrameInA(int^ frameInA)
		{
		}
		public void SetFrameInB(int^ frameInB)
		{
		}
		public void SetJointAxis(int^ jointAxis)
		{
		}
		public void SetPivotInA(int^ pivotInA)
		{
		}
		public void SetPivotInB(int^ pivotInB)
		{
		}
	}
}
