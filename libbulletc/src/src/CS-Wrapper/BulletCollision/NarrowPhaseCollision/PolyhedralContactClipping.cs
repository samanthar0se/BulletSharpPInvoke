namespace BulletSharp
{
	public class PolyhedralContactClipping
	{
		public void ClipFace(int^ pVtxIn, int^ ppVtxOut, int^ planeNormalWS, int planeEqWS)
		{
		}
		public void ClipFaceAgainstHull(int^ separatingNormal, btConvexPolyhedron^ hullA, int^ transA, int^ worldVertsB1, int^ worldVertsB2, int minDist, int maxDist, Result^ resultOut)
		{
		}
		public void ClipHullAgainstHull(int^ separatingNormal1, btConvexPolyhedron^ hullA, btConvexPolyhedron^ hullB, int^ transA, int^ transB, int minDist, int maxDist, int^ worldVertsB1, int^ worldVertsB2, Result^ resultOut)
		{
		}
		public void FindSeparatingAxis(btConvexPolyhedron^ hullA, btConvexPolyhedron^ hullB, int^ transA, int^ transB, int^ sep, Result^ resultOut)
		{
		}
	}
}
