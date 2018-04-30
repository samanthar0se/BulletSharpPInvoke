namespace BulletSharp
{
	public class SimulationIslandManagerMt
	{
		public SimulationIslandManagerMt()
		{
		}
		public void AddBodiesToIslands(int^ collisionWorld)
		{
		}
		public void AddConstraintsToIslands(int )
		{
		}
		public void AddManifoldsToIslands(int^ dispatcher)
		{
		}
		public void AllocateIsland(int id, int numBodies)
		{
		}
		public void BuildAndProcessIslands(int^ dispatcher, int^ collisionWorld, int )
		{
		}
		public void BuildIslands(int^ dispatcher, int^ colWorld)
		{
		}
		public void GetIsland(int id)
		{
		}
		public void GetIslandDispatchFunction()
		{
		}
		public void GetMinimumSolverBatchSize()
		{
		}
		public void InitIslandPools()
		{
		}
		public void MergeIslands()
		{
		}
		public void ParallelIslandDispatch(int )
		{
		}
		public void SerialIslandDispatch(int )
		{
		}
		public void SetIslandDispatchFunction(IslandDispatchFunc func)
		{
		}
		public void SetMinimumSolverBatchSize(int sz)
		{
		}
	public class Island
	{
		public void Append(Island^ other)
		{
		}
	}
	public class IslandCallback
	{
		public void ProcessIsland(int^ bodies, int numBodies, int^ manifolds, int numManifolds, btTypedConstraint^ constraints, int numConstraints, int islandId)
		{
		}
	}
	}
}
