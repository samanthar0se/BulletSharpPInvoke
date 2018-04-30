namespace BulletSharp
{
	public class ConstraintSolverPoolMt
	{
		public ConstraintSolverPoolMt(int numSolvers)
		{
		}
		public ConstraintSolverPoolMt(btConstraintSolver^ solvers, int numSolvers)
		{
		}
		public void GetAndLockThreadSolver()
		{
		}
		public void GetSolverType()
		{
		}
		public void Init(btConstraintSolver^ solvers, int numSolvers)
		{
		}
		public void Reset()
		{
		}
		public void SolveGroup(int^ bodies, int numBodies, btPersistentManifold^ manifolds, int numManifolds, btTypedConstraint^ constraints, int numConstraints, int^ info, btIDebugDraw^ debugDrawer, btDispatcher^ dispatcher)
		{
		}
	public class ThreadSolver
	{
	}
	}
}
