namespace BulletSharp
{
	public class b3PgsJacobiSolver
	{
		public B3PgsJacobiSolver(bool usePgs)
		{
		}
		public void AddFrictionConstraint(b3RigidBodyData^ bodies, b3InertiaData^ inertias, int^ normalAxis, int solverBodyIdA, int solverBodyIdB, int frictionIndex, b3ContactPoint^ cp, int^ rel_pos1, int^ rel_pos2, b3RigidBodyData^ colObj0, b3RigidBodyData^ colObj1, int relaxation, int desiredVelocity, int cfmSlip)
		{
		}
		public void AddRollingFrictionConstraint(b3RigidBodyData^ bodies, b3InertiaData^ inertias, int^ normalAxis, int solverBodyIdA, int solverBodyIdB, int frictionIndex, b3ContactPoint^ cp, int^ rel_pos1, int^ rel_pos2, b3RigidBodyData^ colObj0, b3RigidBodyData^ colObj1, int relaxation, int desiredVelocity, int cfmSlip)
		{
		}
		public void AverageVelocities()
		{
		}
		public void B3_DECLARE_ALIGNED_ALLOCATOR()
		{
		}
		public void B3Rand2()
		{
		}
		public void B3RandInt2(int n)
		{
		}
		public void ConvertContact(b3RigidBodyData^ bodies, b3InertiaData^ inertias, b3Contact4^ manifold, b3ContactSolverInfo^ infoGlobal)
		{
		}
		public void GetContactProcessingThreshold(b3Contact4^ contact)
		{
		}
		public void GetOrInitSolverBody(int bodyIndex, b3RigidBodyData^ bodies, b3InertiaData^ inertias)
		{
		}
		public void GetRandSeed()
		{
		}
		public void InitSolverBody(int bodyIndex, int^ solverBody, b3RigidBodyData^ collisionObject)
		{
		}
		public void Reset()
		{
		}
		public void ResolveSingleConstraintRowGeneric(int^ bodyA, int^ bodyB, int^ contactConstraint)
		{
		}
		public void ResolveSingleConstraintRowGenericSIMD(int^ bodyA, int^ bodyB, int^ contactConstraint)
		{
		}
		public void ResolveSingleConstraintRowLowerLimit(int^ bodyA, int^ bodyB, int^ contactConstraint)
		{
		}
		public void ResolveSingleConstraintRowLowerLimitSIMD(int^ bodyA, int^ bodyB, int^ contactConstraint)
		{
		}
		public void ResolveSplitPenetrationImpulseCacheFriendly(int^ bodyA, int^ bodyB, int^ contactConstraint)
		{
		}
		public void ResolveSplitPenetrationSIMD(int^ bodyA, int^ bodyB, int^ contactConstraint)
		{
		}
		public void RestitutionCurve(int rel_vel, int restitution)
		{
		}
		public void SetFrictionConstraintImpulse(b3RigidBodyData^ bodies, b3InertiaData^ inertias, int^ solverConstraint, int solverBodyIdA, int solverBodyIdB, b3ContactPoint^ cp, b3ContactSolverInfo^ infoGlobal)
		{
		}
		public void SetRandSeed(ulong seed)
		{
		}
		public void SetupContactConstraint(b3RigidBodyData^ bodies, b3InertiaData^ inertias, int^ solverConstraint, int solverBodyIdA, int solverBodyIdB, b3ContactPoint^ cp, b3ContactSolverInfo^ infoGlobal, int^ vel, int^ rel_vel, int^ relaxation, int^ rel_pos1, int^ rel_pos2)
		{
		}
		public void SetupFrictionConstraint(b3RigidBodyData^ bodies, b3InertiaData^ inertias, int^ solverConstraint, int^ normalAxis, int solverBodyIdA, int solverBodyIdB, b3ContactPoint^ cp, int^ rel_pos1, int^ rel_pos2, b3RigidBodyData^ colObj0, b3RigidBodyData^ colObj1, int relaxation, int desiredVelocity, int cfmSlip)
		{
		}
		public void SetupRollingFrictionConstraint(b3RigidBodyData^ bodies, b3InertiaData^ inertias, int^ solverConstraint, int^ normalAxis, int solverBodyIdA, int solverBodyIdB, b3ContactPoint^ cp, int^ rel_pos1, int^ rel_pos2, b3RigidBodyData^ colObj0, b3RigidBodyData^ colObj1, int relaxation, int desiredVelocity, int cfmSlip)
		{
		}
		public void SolveContacts(int numBodies, b3RigidBodyData^ bodies, b3InertiaData^ inertias, int numContacts, b3Contact4^ contacts, int numConstraints, int^ constraints)
		{
		}
		public void SolveGroup(b3RigidBodyData^ bodies, b3InertiaData^ inertias, int numBodies, b3Contact4^ manifoldPtr, int numManifolds, int^ constraints, int numConstraints, b3ContactSolverInfo^ infoGlobal)
		{
		}
		public void SolveGroupCacheFriendlyFinish(b3RigidBodyData^ bodies, b3InertiaData^ inertias, int numBodies, b3ContactSolverInfo^ infoGlobal)
		{
		}
		public void SolveGroupCacheFriendlyIterations(int^ constraints, int numConstraints, b3ContactSolverInfo^ infoGlobal)
		{
		}
		public void SolveGroupCacheFriendlySetup(b3RigidBodyData^ bodies, b3InertiaData^ inertias, int numBodies, b3Contact4^ manifoldPtr, int numManifolds, int^ constraints, int numConstraints, b3ContactSolverInfo^ infoGlobal)
		{
		}
		public void SolveGroupCacheFriendlySplitImpulseIterations(int^ constraints, int numConstraints, b3ContactSolverInfo^ infoGlobal)
		{
		}
		public void SolveSingleIteration(int iteration, int^ constraints, int numConstraints, b3ContactSolverInfo^ infoGlobal)
		{
		}
	}
}
