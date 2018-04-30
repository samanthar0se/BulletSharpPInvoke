namespace BulletSharp
{
	public class SimpleDynamicsWorld
	{
		public SimpleDynamicsWorld(btDispatcher^ dispatcher, int^ pairCache, btConstraintSolver^ constraintSolver, int^ collisionConfiguration)
		{
		}
		public void AddAction(btActionInterface^ action)
		{
		}
		public void AddRigidBody(int^ body)
		{
		}
		public void AddRigidBody(int^ body, int group, int mask)
		{
		}
		public void ClearForces()
		{
		}
		public void DebugDrawWorld()
		{
		}
		public void GetConstraintSolver()
		{
		}
		public void GetGravity()
		{
		}
		public void GetWorldType()
		{
		}
		public void IntegrateTransforms(int timeStep)
		{
		}
		public void PredictUnconstraintMotion(int timeStep)
		{
		}
		public void RemoveAction(btActionInterface^ action)
		{
		}
		public void RemoveCollisionObject(int^ collisionObject)
		{
		}
		public void RemoveRigidBody(int^ body)
		{
		}
		public void SetConstraintSolver(btConstraintSolver^ solver)
		{
		}
		public void SetGravity(int^ gravity)
		{
		}
		public void StepSimulation(int timeStep, int maxSubSteps, int fixedTimeStep)
		{
		}
		public void SynchronizeMotionStates()
		{
		}
		public void UpdateAabbs()
		{
		}
	}
}
