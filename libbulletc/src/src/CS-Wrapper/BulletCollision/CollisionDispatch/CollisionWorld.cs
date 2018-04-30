namespace BulletSharp
{
	public class CollisionWorld
	{
		public CollisionWorld(int^ dispatcher, btBroadphaseInterface^ broadphasePairCache, btCollisionConfiguration^ collisionConfiguration)
		{
		}
		public void AddCollisionObject(btCollisionObject^ collisionObject, int collisionFilterGroup, int collisionFilterMask)
		{
		}
		public void ComputeOverlappingPairs()
		{
		}
		public void ContactPairTest(btCollisionObject^ colObjA, btCollisionObject^ colObjB, ContactResultCallback^ resultCallback)
		{
		}
		public void ContactTest(btCollisionObject^ colObj, ContactResultCallback^ resultCallback)
		{
		}
		public void ConvexSweepTest(btConvexShape^ castShape, int^ from, int^ to, ConvexResultCallback^ resultCallback, int allowedCcdPenetration)
		{
		}
		public void DebugDrawObject(int^ worldTransform, btCollisionShape^ shape, int^ color)
		{
		}
		public void DebugDrawWorld()
		{
		}
		public void GetBroadphase()
		{
		}
		public void GetBroadphase()
		{
		}
		public void GetCollisionObjectArray()
		{
		}
		public void GetCollisionObjectArray()
		{
		}
		public void GetDebugDrawer()
		{
		}
		public void GetDispatcher()
		{
		}
		public void GetDispatcher()
		{
		}
		public void GetDispatchInfo()
		{
		}
		public void GetDispatchInfo()
		{
		}
		public void GetForceUpdateAllAabbs()
		{
		}
		public void GetNumCollisionObjects()
		{
		}
		public void GetPairCache()
		{
		}
		public void ObjectQuerySingle(btConvexShape^ castShape, int^ rayFromTrans, int^ rayToTrans, btCollisionObject^ collisionObject, btCollisionShape^ collisionShape, int^ colObjWorldTransform, ConvexResultCallback^ resultCallback, int allowedPenetration)
		{
		}
		public void ObjectQuerySingleInternal(btConvexShape^ castShape, int^ convexFromTrans, int^ convexToTrans, btCollisionObjectWrapper^ colObjWrap, ConvexResultCallback^ resultCallback, int allowedPenetration)
		{
		}
		public void PerformDiscreteCollisionDetection()
		{
		}
		public void RayTest(int^ rayFromWorld, int^ rayToWorld, RayResultCallback^ resultCallback)
		{
		}
		public void RayTestSingle(int^ rayFromTrans, int^ rayToTrans, btCollisionObject^ collisionObject, btCollisionShape^ collisionShape, int^ colObjWorldTransform, RayResultCallback^ resultCallback)
		{
		}
		public void RayTestSingleInternal(int^ rayFromTrans, int^ rayToTrans, btCollisionObjectWrapper^ collisionObjectWrap, RayResultCallback^ resultCallback)
		{
		}
		public void RemoveCollisionObject(btCollisionObject^ collisionObject)
		{
		}
		public void Serialize(btSerializer^ serializer)
		{
		}
		public void SerializeCollisionObjects(btSerializer^ serializer)
		{
		}
		public void SetBroadphase(btBroadphaseInterface^ pairCache)
		{
		}
		public void SetDebugDrawer(btIDebugDraw^ debugDrawer)
		{
		}
		public void SetForceUpdateAllAabbs(bool forceUpdateAllAabbs)
		{
		}
		public void UpdateAabbs()
		{
		}
		public void UpdateSingleAabb(btCollisionObject^ colObj)
		{
		}
	public class AllHitsRayResultCallback
	{
		public AllHitsRayResultCallback(int^ rayFromWorld, int^ rayToWorld)
		{
		}
		public void AddSingleResult(LocalRayResult^ rayResult, bool normalInWorldSpace)
		{
		}
	}
	public class ClosestConvexResultCallback
	{
		public ClosestConvexResultCallback(int^ convexFromWorld, int^ convexToWorld)
		{
		}
		public void AddSingleResult(LocalConvexResult^ convexResult, bool normalInWorldSpace)
		{
		}
	}
	public class ClosestRayResultCallback
	{
		public ClosestRayResultCallback(int^ rayFromWorld, int^ rayToWorld)
		{
		}
		public void AddSingleResult(LocalRayResult^ rayResult, bool normalInWorldSpace)
		{
		}
	}
	public class ContactResultCallback
	{
		public ContactResultCallback()
		{
		}
		public void AddSingleResult(int^ cp, btCollisionObjectWrapper^ colObj0Wrap, int partId0, int index0, btCollisionObjectWrapper^ colObj1Wrap, int partId1, int index1)
		{
		}
		public void NeedsCollision(btBroadphaseProxy^ proxy0)
		{
		}
	}
	public class ConvexResultCallback
	{
		public ConvexResultCallback()
		{
		}
		public void AddSingleResult(LocalConvexResult^ convexResult, bool normalInWorldSpace)
		{
		}
		public void HasHit()
		{
		}
		public void NeedsCollision(btBroadphaseProxy^ proxy0)
		{
		}
	}
	public class LocalConvexResult
	{
		public LocalConvexResult(btCollisionObject^ hitCollisionObject, LocalShapeInfo^ localShapeInfo, int^ hitNormalLocal, int^ hitPointLocal, int hitFraction)
		{
		}
	}
	public class LocalRayResult
	{
		public LocalRayResult(btCollisionObject^ collisionObject, LocalShapeInfo^ localShapeInfo, int^ hitNormalLocal, int hitFraction)
		{
		}
	}
	public class LocalShapeInfo
	{
	}
	public class RayResultCallback
	{
		public RayResultCallback()
		{
		}
		public void AddSingleResult(LocalRayResult^ rayResult, bool normalInWorldSpace)
		{
		}
		public void HasHit()
		{
		}
		public void NeedsCollision(btBroadphaseProxy^ proxy0)
		{
		}
	}
	}
}
