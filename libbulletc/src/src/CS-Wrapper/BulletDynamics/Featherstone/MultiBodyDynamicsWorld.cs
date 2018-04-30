namespace BulletSharp
{
	public class MultiBodyDynamicsWorld
	{
		public MultiBodyDynamicsWorld(int^ dispatcher, int^ pairCache, btMultiBodyConstraintSolver^ constraintSolver, int^ collisionConfiguration)
		{
		}
		public void AddMultiBody(btMultiBody^ body, int group, int mask)
		{
		}
		public void AddMultiBodyConstraint(btMultiBodyConstraint^ constraint)
		{
		}
		public void ApplyGravity()
		{
		}
		public void CalculateSimulationIslands()
		{
		}
		public void ClearForces()
		{
		}
		public void ClearMultiBodyConstraintForces()
		{
		}
		public void ClearMultiBodyForces()
		{
		}
		public void DebugDrawMultiBodyConstraint(btMultiBodyConstraint^ constraint)
		{
		}
		public void DebugDrawWorld()
		{
		}
		public void ForwardKinematics()
		{
		}
		public void GetMultiBody(int mbIndex)
		{
		}
		public void GetMultiBody(int mbIndex)
		{
		}
		public void GetMultiBodyConstraint(int constraintIndex)
		{
		}
		public void GetMultiBodyConstraint(int constraintIndex)
		{
		}
		public void GetNumMultibodies()
		{
		}
		public void GetNumMultiBodyConstraints()
		{
		}
		public void IntegrateTransforms(int timeStep)
		{
		}
		public void RemoveMultiBody(btMultiBody^ body)
		{
		}
		public void RemoveMultiBodyConstraint(btMultiBodyConstraint^ constraint)
		{
		}
		public void Serialize(int^ serializer)
		{
		}
		public void SerializeMultiBodies(int^ serializer)
		{
		}
		public void SolveConstraints(int^ solverInfo)
		{
		}
		public void UpdateActivationState(int timeStep)
		{
		}
	}
}
