namespace BulletSharp
{
	public class TetrahedronShapeEx
	{
		public TetrahedronShapeEx()
		{
		}
	}
	public class GImpactShapeInterface
	{
		public GImpactShapeInterface()
		{
		}
		public void CalcLocalAABB()
		{
		}
		public void ChildrenHasTransform()
		{
		}
		public void GetBulletTetrahedron(int prim_index, btTetrahedronShapeEx^ tetrahedron)
		{
		}
		public void GetBulletTriangle(int prim_index, btTriangleShapeEx^ triangle)
		{
		}
		public void GetChildShape(int index)
		{
		}
		public void GetChildTransform(int index)
		{
		}
		public void GetNumChildShapes()
		{
		}
		public void LockChildShapes()
		{
		}
		public void NeedsRetrieveTetrahedrons()
		{
		}
		public void NeedsRetrieveTriangles()
		{
		}
		public void ProcessAllTriangles(int^ callback, int^ aabbMin, int^ aabbMax)
		{
		}
		public void ProcessAllTrianglesRay(int^ , int^ , int^ )
		{
		}
		public void RayTest(int^ rayFrom, int^ rayTo, int^ resultCallback)
		{
		}
		public void SetChildTransform(int index, int^ transform)
		{
		}
		public void UnlockChildShapes()
		{
		}
	}
	public class GImpactCompoundShape
	{
		public GImpactCompoundShape(bool children_has_transform)
		{
		}
		public void ChildrenHasTransform()
		{
		}
		public void GetGImpactShapeType()
		{
		}
		public void GetName()
		{
		}
		public void GetPrimitiveManager()
		{
		}
	public class CompoundPrimitiveManager
	{
		public CompoundPrimitiveManager(CompoundPrimitiveManager^ compound)
		{
		}
		public CompoundPrimitiveManager(btGImpactCompoundShape^ compoundShape)
		{
		}
		public CompoundPrimitiveManager()
		{
		}
		public void Get_primitive_box(int prim_index, int^ primbox)
		{
		}
		public void Get_primitive_count()
		{
		}
		public void Get_primitive_triangle(int prim_index, btPrimitiveTriangle^ triangle)
		{
		}
		public void Is_trimesh()
		{
		}
	}
	}
	public class GImpactMeshShapePart
	{
		public GImpactMeshShapePart()
		{
		}
		public GImpactMeshShapePart(int^ meshInterface, int part)
		{
		}
		public void ChildrenHasTransform()
		{
		}
		public void GetBulletTetrahedron(int prim_index, btTetrahedronShapeEx^ tetrahedron)
		{
		}
		public void GetBulletTriangle(int prim_index, btTriangleShapeEx^ triangle)
		{
		}
		public void GetChildShape(int index)
		{
		}
		public void GetChildShape(int index)
		{
		}
		public void GetChildTransform(int index)
		{
		}
		public void GetGImpactShapeType()
		{
		}
		public void GetName()
		{
		}
		public void GetNumChildShapes()
		{
		}
		public void GetPrimitiveManager()
		{
		}
		public void LockChildShapes()
		{
		}
		public void NeedsRetrieveTetrahedrons()
		{
		}
		public void NeedsRetrieveTriangles()
		{
		}
		public void ProcessAllTrianglesRay(int^ callback, int^ rayFrom, int^ rayTo)
		{
		}
		public void SetChildTransform(int index, int^ transform)
		{
		}
		public void UnlockChildShapes()
		{
		}
	public class TrimeshPrimitiveManager
	{
		public TrimeshPrimitiveManager()
		{
		}
		public TrimeshPrimitiveManager(TrimeshPrimitiveManager^ manager)
		{
		}
		public TrimeshPrimitiveManager(int^ meshInterface, int part)
		{
		}
		public void Get_primitive_count()
		{
		}
		public void Is_trimesh()
		{
		}
		public void Lock()
		{
		}
		public void Unlock()
		{
		}
	}
	}
	public class GImpactMeshShape
	{
		public GImpactMeshShape(int^ meshInterface)
		{
		}
		public void BuildMeshParts(int^ meshInterface)
		{
		}
		public void CalcLocalAABB()
		{
		}
		public void CalculateLocalInertia(int mass, int^ inertia)
		{
		}
		public void CalculateSerializeBufferSize()
		{
		}
		public void ChildrenHasTransform()
		{
		}
		public void GetBulletTetrahedron(int prim_index, btTetrahedronShapeEx^ tetrahedron)
		{
		}
		public void GetBulletTriangle(int prim_index, btTriangleShapeEx^ triangle)
		{
		}
		public void GetChildAabb(int child_index, int^ t, int^ aabbMin, int^ aabbMax)
		{
		}
		public void GetChildShape(int index)
		{
		}
		public void GetChildShape(int index)
		{
		}
		public void GetChildTransform(int index)
		{
		}
		public void GetGImpactShapeType()
		{
		}
		public void GetMeshInterface()
		{
		}
		public void GetMeshInterface()
		{
		}
		public void GetMeshPart(int index)
		{
		}
		public void GetMeshPart(int index)
		{
		}
		public void GetMeshPartCount()
		{
		}
		public void GetName()
		{
		}
		public void GetNumChildShapes()
		{
		}
		public void GetPrimitiveManager()
		{
		}
		public void LockChildShapes()
		{
		}
		public void NeedsRetrieveTetrahedrons()
		{
		}
		public void NeedsRetrieveTriangles()
		{
		}
		public void PostUpdate()
		{
		}
		public void ProcessAllTriangles(int^ callback, int^ aabbMin, int^ aabbMax)
		{
		}
		public void ProcessAllTrianglesRay(int^ callback, int^ rayFrom, int^ rayTo)
		{
		}
		public void RayTest(int^ rayFrom, int^ rayTo, int^ resultCallback)
		{
		}
		public void Serialize(void^ dataBuffer, int^ serializer)
		{
		}
		public void SetChildTransform(int index, int^ transform)
		{
		}
		public void SetLocalScaling(int^ scaling)
		{
		}
		public void SetMargin(int margin)
		{
		}
		public void UnlockChildShapes()
		{
		}
	}
	public class GImpactMeshShapeData
	{
	}
}
