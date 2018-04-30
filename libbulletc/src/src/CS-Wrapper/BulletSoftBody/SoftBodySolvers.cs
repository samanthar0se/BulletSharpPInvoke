namespace BulletSharp
{
	public class SoftBodySolver
	{
		public SoftBodySolver()
		{
		}
		public void CheckInitialized()
		{
		}
		public void CopyBackToSoftBodies(bool bMove)
		{
		}
		public void GetNumberOfPositionIterations()
		{
		}
		public void GetNumberOfVelocityIterations()
		{
		}
		public void GetSolverType()
		{
		}
		public void GetTimeScale()
		{
		}
		public void Optimize(int )
		{
		}
		public void PredictMotion(float solverdt)
		{
		}
		public void ProcessCollision(btSoftBody^ , btCollisionObjectWrapper^ )
		{
		}
		public void ProcessCollision(btSoftBody^ , btSoftBody^ )
		{
		}
		public void SetNumberOfPositionIterations(int iterations)
		{
		}
		public void SetNumberOfVelocityIterations(int iterations)
		{
		}
		public void SolveConstraints(float solverdt)
		{
		}
		public void UpdateSoftBodies()
		{
		}
	public enum SolverTypes
	{
		DefaultSolver,
		CpuSolver,
		ClSolver,
		ClSimdSolver,
		DxSolver,
		DxSimdSolver
	}
	}
	public class SoftBodySolverOutput
	{
		public SoftBodySolverOutput()
		{
		}
		public void CopySoftBodyToVertexBuffer(btSoftBody^ softBody, btVertexBufferDescriptor^ vertexBuffer)
		{
		}
	}
}
