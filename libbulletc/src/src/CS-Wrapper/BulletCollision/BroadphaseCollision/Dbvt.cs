namespace BulletSharp
{
	public class DbvtAabbMm
	{
		public void FromCR(int^ c, int r)
		{
		}
		public void FromMM(int^ mi, int^ mx)
		{
		}
		public void FromPoints(int^ pts, int n)
		{
		}
		public void FromPoints(int^ ppts, int n)
		{
		}
	}
	public class DbvtNode
	{
	}
	public class Dbvt
	{
		public Dbvt()
		{
		}
		public void Benchmark()
		{
		}
		public void Clear()
		{
		}
		public void Clone(btDbvt^ dest, IClone^ iclone)
		{
		}
		public void CollideKDOP(btDbvtNode^ root, int^ normals, int^ offsets, int count, ICollide^ policy)
		{
		}
		public void CollideOCL(btDbvtNode^ root, int^ normals, int^ offsets, int^ sortaxis, int count, ICollide^ policy, bool fullsort)
		{
		}
		public void CollideTT(btDbvtNode^ root0, btDbvtNode^ root1, ICollide^ policy)
		{
		}
		public void CollideTTpersistentStack(btDbvtNode^ root0, btDbvtNode^ root1, ICollide^ policy)
		{
		}
		public void CollideTU(btDbvtNode^ root, ICollide^ policy)
		{
		}
		public void CollideTV(btDbvtNode^ root, btDbvtVolume^ volume, ICollide^ policy)
		{
		}
		public void CollideTVNoStackAlloc(btDbvtNode^ root, btDbvtVolume^ volume, int^ stack, ICollide^ policy)
		{
		}
		public void CountLeaves(btDbvtNode^ node)
		{
		}
		public void Empty()
		{
		}
		public void EnumLeaves(btDbvtNode^ root, ICollide^ policy)
		{
		}
		public void EnumNodes(btDbvtNode^ root, ICollide^ policy)
		{
		}
		public void ExtractLeaves(btDbvtNode^ node, int )
		{
		}
		public void Insert(btDbvtVolume^ box, void^ data)
		{
		}
		public void Maxdepth(btDbvtNode^ node)
		{
		}
		public void OptimizeBottomUp()
		{
		}
		public void OptimizeIncremental(int passes)
		{
		}
		public void OptimizeTopDown(int bu_treshold)
		{
		}
		public void RayTest(btDbvtNode^ root, int^ rayFrom, int^ rayTo, ICollide^ policy)
		{
		}
		public void RayTestInternal(btDbvtNode^ root, int^ rayFrom, int^ rayTo, int^ rayDirectionInverse, uint^ signs, int lambda_max, int^ aabbMin, int^ aabbMax, int )
		{
		}
		public void Remove(btDbvtNode^ leaf)
		{
		}
		public void Update(btDbvtNode^ leaf, int lookahead)
		{
		}
		public void Update(btDbvtNode^ leaf, btDbvtVolume^ volume)
		{
		}
		public void Update(btDbvtNode^ leaf, btDbvtVolume^ volume, int^ velocity, int margin)
		{
		}
		public void Update(btDbvtNode^ leaf, btDbvtVolume^ volume, int^ velocity)
		{
		}
		public void Update(btDbvtNode^ leaf, btDbvtVolume^ volume, int margin)
		{
		}
		public void Write(IWriter^ iwriter)
		{
		}
	public class IClone
	{
		public void CloneLeaf(btDbvtNode^ )
		{
		}
	}
	public class ICollide
	{
		public void AllLeaves(btDbvtNode^ )
		{
		}
		public void Descent(btDbvtNode^ )
		{
		}
		public void Process(btDbvtNode^ , btDbvtNode^ )
		{
		}
		public void Process(btDbvtNode^ )
		{
		}
		public void Process(btDbvtNode^ n, int )
		{
		}
	}
	public class IWriter
	{
		public void Prepare(btDbvtNode^ root, int numnodes)
		{
		}
		public void WriteLeaf(btDbvtNode^ , int index, int parent)
		{
		}
		public void WriteNode(btDbvtNode^ , int index, int parent, int child0, int child1)
		{
		}
	}
	public class sStkCLN
	{
		public SStkCLN(btDbvtNode^ n, btDbvtNode^ p)
		{
		}
	}
	public class sStkNN
	{
		public SStkNN()
		{
		}
		public SStkNN(btDbvtNode^ na, btDbvtNode^ nb)
		{
		}
	}
	public class sStkNP
	{
		public SStkNP(btDbvtNode^ n, uint m)
		{
		}
	}
	public class sStkNPS
	{
		public SStkNPS()
		{
		}
		public SStkNPS(btDbvtNode^ n, uint m, int v)
		{
		}
	}
	public enum 
	{
		SimpleStacksize = 64,
		DoubleStacksize = SimpleStacksize*2
	}
	}
}
