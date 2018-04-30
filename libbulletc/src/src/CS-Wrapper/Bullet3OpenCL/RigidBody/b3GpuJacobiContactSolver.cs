namespace BulletSharp
{
	public class b3JacobiSolverInfo
	{
		public B3JacobiSolverInfo()
		{
		}
	}
	public class b3GpuJacobiContactSolver
	{
		public B3GpuJacobiContactSolver(int ctx, int device, int queue, int pairCapacity)
		{
		}
		public void SolveContacts(int numBodies, int bodyBuf, int inertiaBuf, int numContacts, int contactBuf, b3Config^ config, int static0Index)
		{
		}
		public void SolveGroupHost(int^ bodies, int^ inertias, int numBodies, b3Contact4^ manifoldPtr, int numManifolds, b3JacobiSolverInfo^ solverInfo)
		{
		}
	}
}
