namespace BulletSharp
{
	public class b3OverlapCallback
	{
		public void ProcessOverlap(int^ pair)
		{
		}
	}
	public class b3OverlapFilterCallback
	{
		public void NeedBroadphaseCollision(int proxy0, int proxy1)
		{
		}
	}
	public class b3OverlappingPairCache
	{
		public void AddOverlappingPair(int proxy0, int proxy1)
		{
		}
		public void CleanOverlappingPair(int^ pair, b3Dispatcher^ dispatcher)
		{
		}
		public void CleanProxyFromPairs(int proxy, b3Dispatcher^ dispatcher)
		{
		}
		public void FindPair(int proxy0, int proxy1)
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
		public void ProcessAllOverlappingPairs(b3OverlapCallback^ , b3Dispatcher^ dispatcher)
		{
		}
		public void RemoveOverlappingPair(int proxy0, int proxy1, b3Dispatcher^ dispatcher)
		{
		}
		public void RemoveOverlappingPairsContainingProxy(int , b3Dispatcher^ )
		{
		}
		public void SetOverlapFilterCallback(b3OverlapFilterCallback^ callback)
		{
		}
		public void SortOverlappingPairs(b3Dispatcher^ dispatcher)
		{
		}
	}
	public class b3HashedOverlappingPairCache
	{
		public B3HashedOverlappingPairCache()
		{
		}
		public void CleanOverlappingPair(int^ pair, b3Dispatcher^ dispatcher)
		{
		}
		public void FindPair(int proxy0, int proxy1)
		{
		}
		public void GetCount()
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
		public void GrowTables()
		{
		}
		public void InternalAddPair(int proxy0, int proxy1)
		{
		}
		public void ProcessAllOverlappingPairs(b3OverlapCallback^ , b3Dispatcher^ dispatcher)
		{
		}
		public void RemoveOverlappingPair(int proxy0, int proxy1, b3Dispatcher^ dispatcher)
		{
		}
		public void RemoveOverlappingPairsContainingProxy(int proxy, b3Dispatcher^ dispatcher)
		{
		}
		public void SetOverlapFilterCallback(b3OverlapFilterCallback^ callback)
		{
		}
	}
	public class b3SortedOverlappingPairCache
	{
		public B3SortedOverlappingPairCache()
		{
		}
		public void AddOverlappingPair(int proxy0, int proxy1)
		{
		}
		public void CleanOverlappingPair(int^ pair, b3Dispatcher^ dispatcher)
		{
		}
		public void CleanProxyFromPairs(int proxy, b3Dispatcher^ dispatcher)
		{
		}
		public void FindPair(int proxy0, int proxy1)
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
		public void NeedsBroadphaseCollision(int proxy0, int proxy1)
		{
		}
		public void ProcessAllOverlappingPairs(b3OverlapCallback^ , b3Dispatcher^ dispatcher)
		{
		}
		public void RemoveOverlappingPair(int proxy0, int proxy1, b3Dispatcher^ dispatcher)
		{
		}
		public void RemoveOverlappingPairsContainingProxy(int proxy, b3Dispatcher^ dispatcher)
		{
		}
		public void SetOverlapFilterCallback(b3OverlapFilterCallback^ callback)
		{
		}
		public void SortOverlappingPairs(b3Dispatcher^ dispatcher)
		{
		}
	}
	public class b3NullPairCache
	{
		public void AddOverlappingPair(int , int )
		{
		}
		public void CleanOverlappingPair(int^ , b3Dispatcher^ )
		{
		}
		public void CleanProxyFromPairs(int , b3Dispatcher^ )
		{
		}
		public void FindPair(int , int )
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
		public void ProcessAllOverlappingPairs(b3OverlapCallback^ , b3Dispatcher^ )
		{
		}
		public void RemoveOverlappingPair(int , int , b3Dispatcher^ )
		{
		}
		public void RemoveOverlappingPairsContainingProxy(int , b3Dispatcher^ )
		{
		}
		public void SetOverlapFilterCallback(b3OverlapFilterCallback^ )
		{
		}
		public void SortOverlappingPairs(b3Dispatcher^ dispatcher)
		{
		}
	}
}
