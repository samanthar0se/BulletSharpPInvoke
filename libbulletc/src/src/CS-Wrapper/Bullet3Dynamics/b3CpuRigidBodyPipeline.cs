namespace BulletSharp
{
	public class b3CpuRigidBodyPipeline
	{
		public B3CpuRigidBodyPipeline(b3CpuNarrowPhase^ narrowphase, b3DynamicBvhBroadphase^ broadphaseDbvt, b3Config^ config)
		{
		}
		public void AddConstraint(b3TypedConstraint^ constraint)
		{
		}
		public void AllocateCollidable()
		{
		}
		public void CastRays(int )
		{
		}
		public void ComputeContactPoints()
		{
		}
		public void ComputeOverlappingPairs()
		{
		}
		public void CopyConstraintsToHost()
		{
		}
		public void CreateFixedConstraint(int bodyA, int bodyB, float^ pivotInA, float^ pivotInB, float^ relTargetAB, float breakingThreshold)
		{
		}
		public void CreatePoint2PointConstraint(int bodyA, int bodyB, float^ pivotInA, float^ pivotInB, float breakingThreshold)
		{
		}
		public void GetBodyBuffer()
		{
		}
		public void GetNumBodies()
		{
		}
		public void Integrate(float timeStep)
		{
		}
		public void RegisterConvexPolyhedron(b3ConvexUtility^ convex)
		{
		}
		public void RegisterPhysicsInstance(float mass, float^ position, float^ orientation, int collisionShapeIndex, int userData)
		{
		}
		public void RemoveConstraint(b3TypedConstraint^ constraint)
		{
		}
		public void RemoveConstraintByUid(int uid)
		{
		}
		public void Reset()
		{
		}
		public void SetGravity(float^ grav)
		{
		}
		public void SolveContactConstraints()
		{
		}
		public void StepSimulation(float deltaTime)
		{
		}
		public void UpdateAabbWorldSpace()
		{
		}
		public void WriteAllInstancesToGpu()
		{
		}
	}
}
