namespace BulletSharp
{
	public class SymMatrix
	{
		public SymMatrix<T>()
		{
		}
		public SymMatrix<T>(int n, [unexposed type]^ init)
		{
		}
		public void Index(int c, int r)
		{
		}
		public void Resize(int n, [unexposed type]^ init)
		{
		}
	}
	public class SoftBodyCollisionShape
	{
		public SoftBodyCollisionShape(btSoftBody^ backptr)
		{
		}
		public void CalculateLocalInertia(int btScalar, int^ )
		{
		}
		public void GetAabb(int^ t, int^ aabbMin, int^ aabbMax)
		{
		}
		public void GetLocalScaling()
		{
		}
		public void GetName()
		{
		}
		public void ProcessAllTriangles(int^ , int^ , int^ )
		{
		}
		public void SetLocalScaling(int^ )
		{
		}
	}
	public class SoftClusterCollisionShape
	{
		public SoftClusterCollisionShape(Cluster^ cluster)
		{
		}
		public void BatchedUnitVectorGetSupportingVertexWithoutMargin(int^ vectors, int^ supportVerticesOut, int numVectors)
		{
		}
		public void CalculateLocalInertia(int mass, int^ inertia)
		{
		}
		public void GetAabb(int^ t, int^ aabbMin, int^ aabbMax)
		{
		}
		public void GetMargin()
		{
		}
		public void GetName()
		{
		}
		public void GetShapeType()
		{
		}
		public void LocalGetSupportingVertex(int^ vec)
		{
		}
		public void LocalGetSupportingVertexWithoutMargin(int^ vec)
		{
		}
		public void SetMargin(int margin)
		{
		}
	}
	public class Eigen
	{
		public void MulPQ(int^ a, int c, int s, int p, int q)
		{
		}
		public void MulTPQ(int^ a, int c, int s, int p, int q)
		{
		}
		public void System(int^ a, int^ vectors, int^ values)
		{
		}
	}
	public class SoftColliders
	{
	public class ClusterBase
	{
		public ClusterBase()
		{
		}
		public void SolveContact(int^ res, Body ba, Body bb, CJoint^ joint)
		{
		}
	}
	public class CollideCL_RS
	{
		public void Process(int^ leaf)
		{
		}
		public void ProcessColObj(btSoftBody^ ps, int^ colObWrap)
		{
		}
	}
	public class CollideCL_SS
	{
		public void Process(int^ la, int^ lb)
		{
		}
		public void ProcessSoftSoft(btSoftBody^ psa, btSoftBody^ psb)
		{
		}
	}
	public class CollideSDF_RS
	{
		public void DoNode(Node^ n)
		{
		}
		public void Process(int^ leaf)
		{
		}
	}
	public class CollideVF_SS
	{
		public void Process(int^ lnode, int^ lface)
		{
		}
	}
	}
}
