namespace BulletSharp
{
	public class CollisionWorldImporter
	{
		public CollisionWorldImporter(btCollisionWorld^ world)
		{
		}
		public void ConvertAllObjects(btBulletSerializedArrays^ arrays)
		{
		}
		public void ConvertCollisionShape(btCollisionShapeData^ shapeData)
		{
		}
		public void CreateBoxShape(int^ halfExtents)
		{
		}
		public void CreateBvhTriangleMeshShape(btStridingMeshInterface^ trimesh, btOptimizedBvh^ bvh)
		{
		}
		public void CreateCapsuleShapeX(int radius, int height)
		{
		}
		public void CreateCapsuleShapeY(int radius, int height)
		{
		}
		public void CreateCapsuleShapeZ(int radius, int height)
		{
		}
		public void CreateCollisionObject(int^ startTransform, btCollisionShape^ shape, char^ bodyName)
		{
		}
		public void CreateCompoundShape()
		{
		}
		public void CreateConeShapeX(int radius, int height)
		{
		}
		public void CreateConeShapeY(int radius, int height)
		{
		}
		public void CreateConeShapeZ(int radius, int height)
		{
		}
		public void CreateConvexHullShape()
		{
		}
		public void CreateConvexTriangleMeshShape(btStridingMeshInterface^ trimesh)
		{
		}
		public void CreateCylinderShapeX(int radius, int height)
		{
		}
		public void CreateCylinderShapeY(int radius, int height)
		{
		}
		public void CreateCylinderShapeZ(int radius, int height)
		{
		}
		public void CreateMeshInterface(btStridingMeshInterfaceData^ meshData)
		{
		}
		public void CreateMultiSphereShape(int^ positions, int^ radi, int numSpheres)
		{
		}
		public void CreateOptimizedBvh()
		{
		}
		public void CreatePlaneShape(int^ planeNormal, int planeConstant)
		{
		}
		public void CreateScaledTrangleMeshShape(btBvhTriangleMeshShape^ meshShape, int^ localScalingbtBvhTriangleMeshShape)
		{
		}
		public void CreateSphereShape(int radius)
		{
		}
		public void CreateStridingMeshInterfaceData(btStridingMeshInterfaceData^ interfaceData)
		{
		}
		public void CreateTriangleInfoMap()
		{
		}
		public void CreateTriangleMeshContainer()
		{
		}
		public void DeleteAllData()
		{
		}
		public void DuplicateName(char^ name)
		{
		}
		public void GetBvhByIndex(int index)
		{
		}
		public void GetCollisionObjectByName(char^ name)
		{
		}
		public void GetCollisionShapeByIndex(int index)
		{
		}
		public void GetCollisionShapeByName(char^ name)
		{
		}
		public void GetNameForPointer(void^ ptr)
		{
		}
		public void GetNumBvhs()
		{
		}
		public void GetNumCollisionShapes()
		{
		}
		public void GetNumRigidBodies()
		{
		}
		public void GetNumTriangleInfoMaps()
		{
		}
		public void GetRigidBodyByIndex(int index)
		{
		}
		public void GetTriangleInfoMapByIndex(int index)
		{
		}
		public void GetVerboseMode()
		{
		}
		public void SetVerboseMode(int verboseMode)
		{
		}
	}
}
