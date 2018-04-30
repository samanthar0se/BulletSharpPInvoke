namespace BulletSharp
{
	public class OverlapCallback
	{
		public void ProcessOverlap(btBroadphasePair^ pair)
		{
		}
	}
	public class OverlapFilterCallback
	{
		public void NeedBroadphaseCollision(btBroadphaseProxy^ proxy0, btBroadphaseProxy^ proxy1)
		{
		}
	}
	public class OverlappingPairCache
	{
		public void CleanOverlappingPair(btBroadphasePair^ pair, btDispatcher^ dispatcher)
		{
		}
		public void CleanProxyFromPairs(btBroadphaseProxy^ proxy, btDispatcher^ dispatcher)
		{
		}
		public void FindPair(btBroadphaseProxy^ proxy0, btBroadphaseProxy^ proxy1)
		{
		}
		public void GetNumOverlappingPairs()
		{
		}
		public void GetOverlappingPairArray()
		{
		}
		public void GetOverlappingPairArrayPtr()
		{
		}
		public void GetOverlappingPairArrayPtr()
		{
		}
		public void HasDeferredRemoval()
		{
		}
		public void ProcessAllOverlappingPairs(btOverlapCallback^ , btDispatcher^ dispatcher)
		{
		}
		public void SetInternalGhostPairCallback(btOverlappingPairCallback^ ghostPairCallback)
		{
		}
		public void SetOverlapFilterCallback(btOverlapFilterCallback^ callback)
		{
		}
		public void SortOverlappingPairs(btDispatcher^ dispatcher)
		{
		}
	}
	public class SortedOverlappingPairCache
	{
		public SortedOverlappingPairCache()
		{
		}
		public void AddOverlappingPair(btBroadphaseProxy^ proxy0, btBroadphaseProxy^ proxy1)
		{
		}
		public void CleanOverlappingPair(btBroadphasePair^ pair, btDispatcher^ dispatcher)
		{
		}
		public void CleanProxyFromPairs(btBroadphaseProxy^ proxy, btDispatcher^ dispatcher)
		{
		}
		public void FindPair(btBroadphaseProxy^ proxy0, btBroadphaseProxy^ proxy1)
		{
		}
		public void GetNumOverlappingPairs()
		{
		}
		public void GetOverlapFilterCallback()
		{
		}
		public void GetOverlappingPairArray()
		{
		}
		public void GetOverlappingPairArray()
		{
		}
		public void GetOverlappingPairArrayPtr()
		{
		}
		public void GetOverlappingPairArrayPtr()
		{
		}
		public void HasDeferredRemoval()
		{
		}
		public void NeedsBroadphaseCollision(btBroadphaseProxy^ proxy0, btBroadphaseProxy^ proxy1)
		{
		}
		public void ProcessAllOverlappingPairs(btOverlapCallback^ , btDispatcher^ dispatcher)
		{
		}
		public void RemoveOverlappingPair(btBroadphaseProxy^ proxy0, btBroadphaseProxy^ proxy1, btDispatcher^ dispatcher)
		{
		}
		public void RemoveOverlappingPairsContainingProxy(btBroadphaseProxy^ proxy, btDispatcher^ dispatcher)
		{
		}
		public void SetInternalGhostPairCallback(btOverlappingPairCallback^ ghostPairCallback)
		{
		}
		public void SetOverlapFilterCallback(btOverlapFilterCallback^ callback)
		{
		}
		public void SortOverlappingPairs(btDispatcher^ dispatcher)
		{
		}
	}
	public class NullPairCache
	{
		public void AddOverlappingPair(btBroadphaseProxy^ , btBroadphaseProxy^ )
		{
		}
		public void CleanOverlappingPair(btBroadphasePair^ , btDispatcher^ )
		{
		}
		public void CleanProxyFromPairs(btBroadphaseProxy^ , btDispatcher^ )
		{
		}
		public void FindPair(btBroadphaseProxy^ , btBroadphaseProxy^ )
		{
		}
		public void GetNumOverlappingPairs()
		{
		}
		public void GetOverlappingPairArray()
		{
		}
		public void GetOverlappingPairArrayPtr()
		{
		}
		public void GetOverlappingPairArrayPtr()
		{
		}
		public void HasDeferredRemoval()
		{
		}
		public void ProcessAllOverlappingPairs(btOverlapCallback^ , btDispatcher^ )
		{
		}
		public void RemoveOverlappingPair(btBroadphaseProxy^ , btBroadphaseProxy^ , btDispatcher^ )
		{
		}
		public void RemoveOverlappingPairsContainingProxy(btBroadphaseProxy^ , btDispatcher^ )
		{
		}
		public void SetInternalGhostPairCallback(btOverlappingPairCallback^ )
		{
		}
		public void SetOverlapFilterCallback(btOverlapFilterCallback^ )
		{
		}
		public void SortOverlappingPairs(btDispatcher^ dispatcher)
		{
		}
	}
}
