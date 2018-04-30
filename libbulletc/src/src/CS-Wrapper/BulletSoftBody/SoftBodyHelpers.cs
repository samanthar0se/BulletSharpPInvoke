namespace BulletSharp
{
	public class fDrawFlags
	{
	public enum _
	{
		Nodes = 0x0001,
		Links = 0x0002,
		Faces = 0x0004,
		Tetras = 0x0008,
		Normals = 0x0010,
		Contacts = 0x0020,
		Anchors = 0x0040,
		Notes = 0x0080,
		Clusters = 0x0100,
		NodeTree = 0x0200,
		FaceTree = 0x0400,
		ClusterTree = 0x0800,
		Joints = 0x1000,
		Std = Links+Faces+Tetras+Anchors+Notes+Joints,
		StdTetra = Std-Faces+Tetras
	}
	}
	public class SoftBodyHelpers
	{
		public void CalculateUV(int resx, int resy, int ix, int iy, int id)
		{
		}
		public void CreateEllipsoid(btSoftBodyWorldInfo^ worldInfo, int^ center, int^ radius, int res)
		{
		}
		public void CreateFromConvexHull(btSoftBodyWorldInfo^ worldInfo, int^ vertices, int nvertices, bool randomizeConstraints)
		{
		}
		public void CreateFromTetGenData(btSoftBodyWorldInfo^ worldInfo, char^ ele, char^ face, char^ node, bool bfacelinks, bool btetralinks, bool bfacesfromtetras)
		{
		}
		public void CreateFromTriMesh(btSoftBodyWorldInfo^ worldInfo, int^ vertices, int^ triangles, int ntriangles, bool randomizeConstraints)
		{
		}
		public void CreatePatch(btSoftBodyWorldInfo^ worldInfo, int^ corner00, int^ corner10, int^ corner01, int^ corner11, int resx, int resy, int fixeds, bool gendiags)
		{
		}
		public void CreatePatchUV(btSoftBodyWorldInfo^ worldInfo, int^ corner00, int^ corner10, int^ corner01, int^ corner11, int resx, int resy, int fixeds, bool gendiags, float^ tex_coords)
		{
		}
		public void CreateRope(btSoftBodyWorldInfo^ worldInfo, int^ from, int^ to, int res, int fixeds)
		{
		}
		public void Draw(btSoftBody^ psb, int^ idraw, int drawflags)
		{
		}
		public void DrawClusterTree(btSoftBody^ psb, int^ idraw, int mindepth, int maxdepth)
		{
		}
		public void DrawFaceTree(btSoftBody^ psb, int^ idraw, int mindepth, int maxdepth)
		{
		}
		public void DrawFrame(btSoftBody^ psb, int^ idraw)
		{
		}
		public void DrawInfos(btSoftBody^ psb, int^ idraw, bool masses, bool areas, bool stress)
		{
		}
		public void DrawNodeTree(btSoftBody^ psb, int^ idraw, int mindepth, int maxdepth)
		{
		}
		public void ReoptimizeLinkOrder(btSoftBody^ psb)
		{
		}
	}
}
