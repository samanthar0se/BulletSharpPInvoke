namespace BulletSharp
{
	public class DynamicsWorld
	{
		public DynamicsWorld(int^ dispatcher, int^ broadphase, int^ collisionConfiguration)
		{
		}
		public void AddAction(btActionInterface^ action)
		{
		}
		public void AddCharacter(btActionInterface^ character)
		{
		}
		public void AddConstraint(btTypedConstraint^ constraint, bool disableCollisionsBetweenLinkedBodies)
		{
		}
		public void AddRigidBody(int^ body)
		{
		}
		public void AddRigidBody(int^ body, int group, int mask)
		{
		}
		public void AddVehicle(btActionInterface^ vehicle)
		{
		}
		public void ClearForces()
		{
		}
		public void DebugDrawWorld()
		{
		}
		public void GetConstraint(int index)
		{
		}
		public void GetConstraint(int index)
		{
		}
		public void GetConstraintSolver()
		{
		}
		public void GetGravity()
		{
		}
		public void GetNumConstraints()
		{
		}
		public void GetSolverInfo()
		{
		}
		public void GetSolverInfo()
		{
		}
		public void GetWorldType()
		{
		}
		public void GetWorldUserInfo()
		{
		}
		public void RemoveAction(btActionInterface^ action)
		{
		}
		public void RemoveCharacter(btActionInterface^ character)
		{
		}
		public void RemoveConstraint(btTypedConstraint^ constraint)
		{
		}
		public void RemoveRigidBody(int^ body)
		{
		}
		public void RemoveVehicle(btActionInterface^ vehicle)
		{
		}
		public void SetConstraintSolver(btConstraintSolver^ solver)
		{
		}
		public void SetGravity(int^ gravity)
		{
		}
		public void SetInternalTickCallback(btInternalTickCallback cb, void^ worldUserInfo, bool isPreTick)
		{
		}
		public void SetWorldUserInfo(void^ worldUserInfo)
		{
		}
		public void StepSimulation(int timeStep, int maxSubSteps, int fixedTimeStep)
		{
		}
		public void SynchronizeMotionStates()
		{
		}
	}
	public class DynamicsWorldDoubleData
	{
	}
	public class DynamicsWorldFloatData
	{
	}
}
