namespace BulletSharp
{
	public class SoftRigidDynamicsWorld
	{
		public SoftRigidDynamicsWorld(btDispatcher^ dispatcher, btBroadphaseInterface^ pairCache, int^ constraintSolver, int^ collisionConfiguration, btSoftBodySolver^ softBodySolver)
		{
		}
		public void AddSoftBody(btSoftBody^ body, int collisionFilterGroup, int collisionFilterMask)
		{
		}
		public void DebugDrawWorld()
		{
		}
		public void GetDrawFlags()
		{
		}
		public void GetSoftBodyArray()
		{
		}
		public void GetSoftBodyArray()
		{
		}
		public void GetWorldInfo()
		{
		}
		public void GetWorldInfo()
		{
		}
		public void GetWorldType()
		{
		}
		public void InternalSingleStepSimulation(int timeStep)
		{
		}
		public void PredictUnconstraintMotion(int timeStep)
		{
		}
		public void RayTest(int^ rayFromWorld, int^ rayToWorld, int^ resultCallback)
		{
		}
		public void RayTestSingle(int^ rayFromTrans, int^ rayToTrans, int^ collisionObject, int^ collisionShape, int^ colObjWorldTransform, int^ resultCallback)
		{
		}
		public void RemoveCollisionObject(int^ collisionObject)
		{
		}
		public void RemoveSoftBody(btSoftBody^ body)
		{
		}
		public void Serialize(btSerializer^ serializer)
		{
		}
		public void SerializeSoftBodies(btSerializer^ serializer)
		{
		}
		public void SetDrawFlags(int f)
		{
		}
		public void SolveSoftBodiesConstraints(int timeStep)
		{
		}
	}
}
