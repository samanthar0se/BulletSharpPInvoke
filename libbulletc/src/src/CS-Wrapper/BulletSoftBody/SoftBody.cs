namespace BulletSharp
{
	public class SoftBodyWorldInfo
	{
		public SoftBodyWorldInfo()
		{
		}
	}
	public class SoftBody
	{
		public SoftBody(btSoftBodyWorldInfo^ worldInfo, int node_count, int^ x, int^ m)
		{
		}
		public SoftBody(btSoftBodyWorldInfo^ worldInfo)
		{
		}
		public void AddAeroForceToFace(int^ windVelocity, int faceIndex)
		{
		}
		public void AddAeroForceToNode(int^ windVelocity, int nodeIndex)
		{
		}
		public void AddForce(int^ force)
		{
		}
		public void AddForce(int^ force, int node)
		{
		}
		public void AddVelocity(int^ velocity)
		{
		}
		public void AddVelocity(int^ velocity, int node)
		{
		}
		public void AppendAnchor(int node, int^ body, bool disableCollisionBetweenLinkedBodies, int influence)
		{
		}
		public void AppendAnchor(int node, int^ body, int^ localPivot, bool disableCollisionBetweenLinkedBodies, int influence)
		{
		}
		public void AppendAngularJoint(Specs^ specs, Cluster^ body0, Body body1)
		{
		}
		public void AppendAngularJoint(Specs^ specs, Body body)
		{
		}
		public void AppendAngularJoint(Specs^ specs, btSoftBody^ body)
		{
		}
		public void AppendFace(int model, Material^ mat)
		{
		}
		public void AppendFace(int node0, int node1, int node2, Material^ mat)
		{
		}
		public void AppendLinearJoint(Specs^ specs, Cluster^ body0, Body body1)
		{
		}
		public void AppendLinearJoint(Specs^ specs, Body body)
		{
		}
		public void AppendLinearJoint(Specs^ specs, btSoftBody^ body)
		{
		}
		public void AppendLink(int model, Material^ mat)
		{
		}
		public void AppendLink(int node0, int node1, Material^ mat, bool bcheckexist)
		{
		}
		public void AppendLink(Node^ node0, Node^ node1, Material^ mat, bool bcheckexist)
		{
		}
		public void AppendMaterial()
		{
		}
		public void AppendNode(int^ x, int m)
		{
		}
		public void AppendNote(char^ text, int^ o, int^ c, Node^ n0, Node^ n1, Node^ n2, Node^ n3)
		{
		}
		public void AppendNote(char^ text, int^ o, Node^ feature)
		{
		}
		public void AppendNote(char^ text, int^ o, int^ feature)
		{
		}
		public void AppendNote(char^ text, int^ o, Face^ feature)
		{
		}
		public void AppendTetra(int model, Material^ mat)
		{
		}
		public void AppendTetra(int node0, int node1, int node2, int node3, Material^ mat)
		{
		}
		public void ApplyClusters(bool drift)
		{
		}
		public void ApplyForces()
		{
		}
		public void ATTRIBUTE_ALIGNED16()
		{
		}
		public void CalculateSerializeBufferSize()
		{
		}
		public void CheckContact(int^ colObjWrap, int^ x, int margin, sCti^ cti)
		{
		}
		public void CheckFace(int node0, int node1, int node2)
		{
		}
		public void CheckLink(int node0, int node1)
		{
		}
		public void CheckLink(Node^ node0, Node^ node1)
		{
		}
		public void CleanupClusters()
		{
		}
		public void ClusterAImpulse(Cluster^ cluster, Impulse^ impulse)
		{
		}
		public void ClusterCom(Cluster^ cluster)
		{
		}
		public void ClusterCom(int cluster)
		{
		}
		public void ClusterCount()
		{
		}
		public void ClusterDAImpulse(Cluster^ cluster, int^ impulse)
		{
		}
		public void ClusterDCImpulse(Cluster^ cluster, int^ impulse)
		{
		}
		public void ClusterDImpulse(Cluster^ cluster, int^ rpos, int^ impulse)
		{
		}
		public void ClusterImpulse(Cluster^ cluster, int^ rpos, Impulse^ impulse)
		{
		}
		public void ClusterVAImpulse(Cluster^ cluster, int^ impulse)
		{
		}
		public void ClusterVelocity(Cluster^ cluster, int^ rpos)
		{
		}
		public void ClusterVImpulse(Cluster^ cluster, int^ rpos, int^ impulse)
		{
		}
		public void CutLink(int node0, int node1, int position)
		{
		}
		public void CutLink(Node^ node0, Node^ node1, int position)
		{
		}
		public void DampClusters()
		{
		}
		public void DefaultCollisionHandler(int^ pcoWrap)
		{
		}
		public void DefaultCollisionHandler(btSoftBody^ psb)
		{
		}
		public void EvaluateCom()
		{
		}
		public void GenerateBendingConstraints(int distance, Material^ mat)
		{
		}
		public void GenerateClusters(int k, int maxiterations)
		{
		}
		public void GetAabb(int^ aabbMin, int^ aabbMax)
		{
		}
		public void GetMass(int node)
		{
		}
		public void GetRestLengthScale()
		{
		}
		public void GetSoftBodySolver()
		{
		}
		public void GetSoftBodySolver()
		{
		}
		public void GetSolver(_ solver)
		{
		}
		public void GetSolver(_ solver)
		{
		}
		public void GetTotalMass()
		{
		}
		public void GetVolume()
		{
		}
		public void GetWindVelocity()
		{
		}
		public void GetWorldInfo()
		{
		}
		public void IndicesToPointers(int^ map)
		{
		}
		public void InitDefaults()
		{
		}
		public void InitializeClusters()
		{
		}
		public void InitializeFaceTree()
		{
		}
		public void IntegrateMotion()
		{
		}
		public void PointersToIndices()
		{
		}
		public void PredictMotion(int dt)
		{
		}
		public void PrepareClusters(int iterations)
		{
		}
		public void PSolve_Anchors(btSoftBody^ psb, int kst, int ti)
		{
		}
		public void PSolve_Links(btSoftBody^ psb, int kst, int ti)
		{
		}
		public void PSolve_RContacts(btSoftBody^ psb, int kst, int ti)
		{
		}
		public void PSolve_SContacts(btSoftBody^ psb, int btScalar, int ti)
		{
		}
		public void RandomizeConstraints()
		{
		}
		public void RayTest(int^ rayFrom, int^ rayTo, sRayCast^ results)
		{
		}
		public void RayTest(int^ rayFrom, int^ rayTo, int^ mint, _^ feature, int^ index, bool bcountonly)
		{
		}
		public void Refine(ImplicitFn^ ifn, int accurary, bool cut)
		{
		}
		public void ReleaseCluster(int index)
		{
		}
		public void ReleaseClusters()
		{
		}
		public void ResetLinkRestLengths()
		{
		}
		public void Rotate(int^ rot)
		{
		}
		public void Scale(int^ scl)
		{
		}
		public void Serialize(void^ dataBuffer, btSerializer^ serializer)
		{
		}
		public void SetCollisionShape(int^ collisionShape)
		{
		}
		public void SetMass(int node, int mass)
		{
		}
		public void SetPose(bool bvolume, bool bframe)
		{
		}
		public void SetRestLengthScale(int restLength)
		{
		}
		public void SetSoftBodySolver(btSoftBodySolver^ softBodySolver)
		{
		}
		public void SetSolver(_ preset)
		{
		}
		public void SetTotalDensity(int density)
		{
		}
		public void SetTotalMass(int mass, bool fromfaces)
		{
		}
		public void SetVelocity(int^ velocity)
		{
		}
		public void SetVolumeDensity(int density)
		{
		}
		public void SetVolumeMass(int mass)
		{
		}
		public void SetWindVelocity(int^ velocity)
		{
		}
		public void SolveClusters(int )
		{
		}
		public void SolveClusters(int sor)
		{
		}
		public void SolveCommonConstraints(btSoftBody^ bodies, int count, int iterations)
		{
		}
		public void SolveConstraints()
		{
		}
		public void StaticSolve(int iterations)
		{
		}
		public void Transform(int^ trs)
		{
		}
		public void Translate(int^ trs)
		{
		}
		public void Upcast(int^ colObj)
		{
		}
		public void Upcast(int^ colObj)
		{
		}
		public void UpdateArea(bool averageArea)
		{
		}
		public void UpdateBounds()
		{
		}
		public void UpdateClusters()
		{
		}
		public void UpdateConstants()
		{
		}
		public void UpdateLinkConstants()
		{
		}
		public void UpdateNormals()
		{
		}
		public void UpdatePose()
		{
		}
		public void VSolve_Links(btSoftBody^ psb, int kst)
		{
		}
	public class AJoint
	{
		public void Prepare(int dt, int iterations)
		{
		}
		public void Solve(int dt, int sor)
		{
		}
		public void Terminate(int dt)
		{
		}
		public void Type()
		{
		}
	public class IControl
	{
		public void Default()
		{
		}
		public void Prepare(AJoint^ )
		{
		}
		public void Speed(AJoint^ , int current)
		{
		}
	}
	public class Specs
	{
		public Specs()
		{
		}
	}
	}
	public class Anchor
	{
	}
	public class Body
	{
		public Body()
		{
		}
		public Body(Cluster^ p)
		{
		}
		public Body(int^ colObj)
		{
		}
		public void Activate()
		{
		}
		public void AngularVelocity(int^ rpos)
		{
		}
		public void AngularVelocity()
		{
		}
		public void ApplyAImpulse(Impulse^ impulse)
		{
		}
		public void ApplyDAImpulse(int^ impulse)
		{
		}
		public void ApplyDCImpulse(int^ impulse)
		{
		}
		public void ApplyDImpulse(int^ impulse, int^ rpos)
		{
		}
		public void ApplyImpulse(Impulse^ impulse, int^ rpos)
		{
		}
		public void ApplyVAImpulse(int^ impulse)
		{
		}
		public void ApplyVImpulse(int^ impulse, int^ rpos)
		{
		}
		public void InvMass()
		{
		}
		public void InvWorldInertia()
		{
		}
		public void LinearVelocity()
		{
		}
		public void Velocity(int^ rpos)
		{
		}
		public void Xform()
		{
		}
	}
	public class CJoint
	{
		public void Prepare(int dt, int iterations)
		{
		}
		public void Solve(int dt, int sor)
		{
		}
		public void Terminate(int dt)
		{
		}
		public void Type()
		{
		}
	}
	public class Cluster
	{
		public Cluster()
		{
		}
	}
	public class Config
	{
	}
	public class eAeroModel
	{
	public enum _
	{
		VPoint,
		VTwoSided,
		VTwoSidedLiftDrag,
		VOneSided,
		FTwoSided,
		FTwoSidedLiftDrag,
		FOneSided,
		End
	}
	}
	public class eFeature
	{
	public enum _
	{
		None,
		Node,
		Link,
		Face,
		Tetra,
		End
	}
	}
	public class Element
	{
		public Element()
		{
		}
	}
	public class ePSolver
	{
	public enum _
	{
		Linear,
		Anchors,
		RContacts,
		SContacts,
		End
	}
	}
	public class eSolverPresets
	{
	public enum _
	{
		Positions,
		Velocities,
		Default = Positions,
		End
	}
	}
	public class eVSolver
	{
	public enum _
	{
		Linear,
		End
	}
	}
	public class Face
	{
	}
	public class fCollision
	{
	public enum _
	{
		RVSmask = 0x000f,
		SdfRs = 0x0001,
		ClRs = 0x0002,
		SVSmask = 0x0030,
		VfSs = 0x0010,
		ClSs = 0x0020,
		ClSelf = 0x0040,
		Default = SdfRs,
		End
	}
	}
	public class Feature
	{
	}
	public class fMaterial
	{
	public enum _
	{
		DebugDraw = 0x0001,
		Default = DebugDraw,
		End
	}
	}
	public class ImplicitFn
	{
		public void Eval(int^ x)
		{
		}
	}
	public class Impulse
	{
		public Impulse()
		{
		}
	}
	public class Joint
	{
		public Joint()
		{
		}
		public void Prepare(int dt, int iterations)
		{
		}
		public void Solve(int dt, int sor)
		{
		}
		public void Terminate(int dt)
		{
		}
		public void Type()
		{
		}
	public class eType
	{
	public enum _
	{
		Linear = 0,
		Angular,
		Contact
	}
	}
	public class Specs
	{
		public Specs()
		{
		}
	}
	}
	public class LJoint
	{
		public void Prepare(int dt, int iterations)
		{
		}
		public void Solve(int dt, int sor)
		{
		}
		public void Terminate(int dt)
		{
		}
		public void Type()
		{
		}
	public class Specs
	{
	}
	}
	public class Material
	{
	}
	public class Node
	{
	}
	public class Note
	{
	}
	public class Pose
	{
	}
	public class RayFromToCaster
	{
		public RayFromToCaster(int^ rayFrom, int^ rayTo, int mxt)
		{
		}
		public void Process(int^ leaf)
		{
		}
		public void RayFromToTriangle(int^ rayFrom, int^ rayTo, int^ rayNormalizedDirection, int^ a, int^ b, int^ c, int maxt)
		{
		}
	}
	public class RContact
	{
	}
	public class SContact
	{
	}
	public class sCti
	{
	}
	public class sMedium
	{
	}
	public class SolverState
	{
	}
	public class sRayCast
	{
	}
	public class Tetra
	{
	}
	}
}
