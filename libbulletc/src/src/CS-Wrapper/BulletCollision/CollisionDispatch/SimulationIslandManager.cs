namespace BulletSharp
{
	public class SimulationIslandManager
	{
		public SimulationIslandManager()
		{
		}
		public void BuildAndProcessIslands(btDispatcher^ dispatcher, btCollisionWorld^ collisionWorld, IslandCallback^ callback)
		{
		}
		public void BuildIslands(btDispatcher^ dispatcher, btCollisionWorld^ colWorld)
		{
		}
		public void FindUnions(btDispatcher^ dispatcher, btCollisionWorld^ colWorld)
		{
		}
		public void GetSplitIslands()
		{
		}
		public void GetUnionFind()
		{
		}
		public void InitUnionFind(int n)
		{
		}
		public void SetSplitIslands(bool doSplitIslands)
		{
		}
		public void StoreIslandActivationState(btCollisionWorld^ world)
		{
		}
		public void UpdateActivationState(btCollisionWorld^ colWorld, btDispatcher^ dispatcher)
		{
		}
	public class IslandCallback
	{
		public void ProcessIsland(btCollisionObject^ bodies, int numBodies, btPersistentManifold^ manifolds, int numManifolds, int islandId)
		{
		}
	}
	}
}
