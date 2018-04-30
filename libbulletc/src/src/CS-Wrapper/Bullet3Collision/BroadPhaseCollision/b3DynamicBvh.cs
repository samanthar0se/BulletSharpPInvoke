namespace BulletSharp
{
	public class b3DbvtAabbMm
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
	public class b3DbvtNode
	{
	}
	public class b3DynamicBvh
	{
		public B3DynamicBvh()
		{
		}
		public void Benchmark()
		{
		}
		public void Clear()
		{
		}
		public void Clone(b3DynamicBvh^ dest, IClone^ iclone)
		{
		}
		public void CollideKDOP(b3DbvtNode^ root, int^ normals, int^ offsets, int count, ICollide^ policy)
		{
		}
		public void CollideOCL(b3DbvtNode^ root, int^ normals, int^ offsets, int^ sortaxis, int count, ICollide^ policy, bool fullsort)
		{
		}
		public void CollideTT(b3DbvtNode^ root0, b3DbvtNode^ root1, ICollide^ policy)
		{
		}
		public void CollideTTpersistentStack(b3DbvtNode^ root0, b3DbvtNode^ root1, ICollide^ policy)
		{
		}
		public void CollideTU(b3DbvtNode^ root, ICollide^ policy)
		{
		}
		public void CollideTV(b3DbvtNode^ root, b3DbvtVolume^ volume, ICollide^ policy)
		{
		}
		public void CountLeaves(b3DbvtNode^ node)
		{
		}
		public void Empty()
		{
		}
		public void EnumLeaves(b3DbvtNode^ root, ICollide^ policy)
		{
		}
		public void EnumNodes(b3DbvtNode^ root, ICollide^ policy)
		{
		}
		public void ExtractLeaves(b3DbvtNode^ node, int )
		{
		}
		public void Insert(b3DbvtVolume^ box, void^ data)
		{
		}
		public void Maxdepth(b3DbvtNode^ node)
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
		public void RayTest(b3DbvtNode^ root, int^ rayFrom, int^ rayTo, ICollide^ policy)
		{
		}
		public void RayTestInternal(b3DbvtNode^ root, int^ rayFrom, int^ rayTo, int^ rayDirectionInverse, uint^ signs, int lambda_max, int^ aabbMin, int^ aabbMax, ICollide^ policy)
		{
		}
		public void Remove(b3DbvtNode^ leaf)
		{
		}
		public void Update(b3DbvtNode^ leaf, int lookahead)
		{
		}
		public void Update(b3DbvtNode^ leaf, b3DbvtVolume^ volume)
		{
		}
		public void Update(b3DbvtNode^ leaf, b3DbvtVolume^ volume, int^ velocity, int margin)
		{
		}
		public void Update(b3DbvtNode^ leaf, b3DbvtVolume^ volume, int^ velocity)
		{
		}
		public void Update(b3DbvtNode^ leaf, b3DbvtVolume^ volume, int margin)
		{
		}
		public void Write(IWriter^ iwriter)
		{
		}
	public class IClone
	{
		public void CloneLeaf(b3DbvtNode^ )
		{
		}
	}
	public class ICollide
	{
		public void AllLeaves(b3DbvtNode^ )
		{
		}
		public void Descent(b3DbvtNode^ )
		{
		}
		public void Process(b3DbvtNode^ , b3DbvtNode^ )
		{
		}
		public void Process(b3DbvtNode^ )
		{
		}
		public void Process(b3DbvtNode^ n, int )
		{
		}
	}
	public class IWriter
	{
		public void Prepare(b3DbvtNode^ root, int numnodes)
		{
		}
		public void WriteLeaf(b3DbvtNode^ , int index, int parent)
		{
		}
		public void WriteNode(b3DbvtNode^ , int index, int parent, int child0, int child1)
		{
		}
	}
	public class sStkCLN
	{
		public SStkCLN(b3DbvtNode^ n, b3DbvtNode^ p)
		{
		}
	}
	public class sStkNN
	{
		public SStkNN()
		{
		}
		public SStkNN(b3DbvtNode^ na, b3DbvtNode^ nb)
		{
		}
	}
	public class sStkNP
	{
		public SStkNP(b3DbvtNode^ n, uint m)
		{
		}
	}
	public class sStkNPS
	{
		public SStkNPS()
		{
		}
		public SStkNPS(b3DbvtNode^ n, uint m, int v)
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
