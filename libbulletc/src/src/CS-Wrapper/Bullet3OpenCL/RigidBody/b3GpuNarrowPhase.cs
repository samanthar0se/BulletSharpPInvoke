namespace BulletSharp
{
	public class b3GpuNarrowPhase
	{
		public B3GpuNarrowPhase(int vtx, int dev, int q, b3Config^ config)
		{
		}
		public void AllocateCollidable()
		{
		}
		public void ComputeContacts(int broadphasePairs, int numBroadphasePairs, int aabbsWorldSpace, int numObjects)
		{
		}
		public void GetAabbLocalSpaceBufferGpu()
		{
		}
		public void GetBodiesCpu()
		{
		}
		public void GetBodiesGpu()
		{
		}
		public void GetBodyInertiasGpu()
		{
		}
		public void GetCollidableCpu(int collidableIndex)
		{
		}
		public void GetCollidableCpu(int collidableIndex)
		{
		}
		public void GetCollidablesCpu()
		{
		}
		public void GetCollidablesGpu()
		{
		}
		public void GetContactsCPU()
		{
		}
		public void GetContactsGpu()
		{
		}
		public void GetInternalData()
		{
		}
		public void GetInternalData()
		{
		}
		public void GetLocalSpaceAabb(int collidableIndex)
		{
		}
		public void GetLocalSpaceAabbsCpu()
		{
		}
		public void GetNumBodiesGpu()
		{
		}
		public void GetNumBodyInertiasGpu()
		{
		}
		public void GetNumCollidablesGpu()
		{
		}
		public void GetNumContactsGpu()
		{
		}
		public void GetNumRigidBodies()
		{
		}
		public void GetObjectTransformFromCpu(float^ position, float^ orientation, int bodyIndex)
		{
		}
		public void GetStatic0Index()
		{
		}
		public void ReadbackAllBodiesToCpu()
		{
		}
		public void RegisterCompoundShape(int )
		{
		}
		public void RegisterConcaveMesh(int )
		{
		}
		public void RegisterConcaveMeshShape(int )
		{
		}
		public void RegisterConvexHullShape(b3ConvexUtility^ utilPtr)
		{
		}
		public void RegisterConvexHullShape(float^ vertices, int strideInBytes, int numVertices, float^ scaling)
		{
		}
		public void RegisterConvexHullShapeInternal(b3ConvexUtility^ convexPtr, int^ col)
		{
		}
		public void RegisterFace(int^ faceNormal, float faceConstant)
		{
		}
		public void RegisterPlaneShape(int^ planeNormal, float planeConstant)
		{
		}
		public void RegisterRigidBody(int collidableIndex, float mass, float^ position, float^ orientation, float^ aabbMin, float^ aabbMax, bool writeToGpu)
		{
		}
		public void RegisterSphereShape(float radius)
		{
		}
		public void Reset()
		{
		}
		public void SetObjectTransform(float^ position, float^ orientation, int bodyIndex)
		{
		}
		public void SetObjectTransformCpu(float^ position, float^ orientation, int bodyIndex)
		{
		}
		public void SetObjectVelocityCpu(float^ linVel, float^ angVel, int bodyIndex)
		{
		}
		public void WriteAllBodiesToGpu()
		{
		}
	}
}
