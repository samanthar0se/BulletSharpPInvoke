namespace BulletSharp
{
	public class HullResult
	{
		public HullResult()
		{
		}
	}
	public class HullDesc
	{
		public HullDesc()
		{
		}
		public HullDesc(HullFlag flag, uint vcount, btVector3^ vertices, uint stride)
		{
		}
		public void ClearHullFlag(HullFlag flag)
		{
		}
		public void HasHullFlag(HullFlag flag)
		{
		}
		public void SetHullFlag(HullFlag flag)
		{
		}
	}
	public class Plane
	{
		public Plane(btVector3^ n, float d)
		{
		}
		public Plane()
		{
		}
	}
	public class ConvexH
	{
		public ConvexH()
		{
		}
		public ConvexH(int vertices_size, int edges_size, int facets_size)
		{
		}
	public class HalfEdge
	{
		public HalfEdge()
		{
		}
		public HalfEdge(short _ea, byte _v, byte _p)
		{
		}
	}
	}
	public class int4
	{
		public Int4()
		{
		}
		public Int4(int _x, int _y, int _z, int _w)
		{
		}
	}
	public class PHullResult
	{
		public PHullResult()
		{
		}
	}
	public class HullLibrary
	{
		public void AllocateTriangle(int a, int b, int c)
		{
		}
		public void B2bfix(btHullTriangle^ s, btHullTriangle^ t)
		{
		}
		public void BringOutYourDead(btVector3^ verts, uint vcount, btVector3^ overts, uint^ ocount, uint^ indices, uint indexcount)
		{
		}
		public void Calchull(btVector3^ verts, int verts_count, TUIntArray^ tris_out, int^ tris_count, int vlimit)
		{
		}
		public void Calchullgen(btVector3^ verts, int verts_count, int vlimit)
		{
		}
		public void Checkit(btHullTriangle^ t)
		{
		}
		public void CleanupVertices(uint svcount, btVector3^ svertices, uint stride, uint^ vcount, btVector3^ vertices, float normalepsilon, btVector3^ scale)
		{
		}
		public void ComputeHull(uint vcount, btVector3^ vertices, PHullResult^ result, uint vlimit)
		{
		}
		public void ConvexHCrop(ConvexH^ convex, btPlane^ slice)
		{
		}
		public void CreateConvexHull(HullDesc^ desc, HullResult^ result)
		{
		}
		public void DeAllocateTriangle(btHullTriangle^ )
		{
		}
		public void Extrudable(float epsilon)
		{
		}
		public void Extrude(btHullTriangle^ t0, int v)
		{
		}
		public void FindSimplex(btVector3^ verts, int verts_count, btAlignedObjectArray^ allow)
		{
		}
		public void ReleaseResult(HullResult^ result)
		{
		}
		public void Removeb2b(btHullTriangle^ s, btHullTriangle^ t)
		{
		}
		public void Test_cube()
		{
		}
	}
}
