namespace BulletSharp
{
	public class IDebugDraw
	{
		public void ClearLines()
		{
		}
		public void Draw3dText(btVector3^ location, char^ textString)
		{
		}
		public void DrawAabb(btVector3^ from, btVector3^ to, btVector3^ color)
		{
		}
		public void DrawArc(btVector3^ center, btVector3^ normal, btVector3^ axis, float radiusA, float radiusB, float minAngle, float maxAngle, btVector3^ color, bool drawSect, float stepDegrees)
		{
		}
		public void DrawBox(btVector3^ bbMin, btVector3^ bbMax, btVector3^ color)
		{
		}
		public void DrawBox(btVector3^ bbMin, btVector3^ bbMax, btTransform^ trans, btVector3^ color)
		{
		}
		public void DrawCapsule(float radius, float halfHeight, int upAxis, btTransform^ transform, btVector3^ color)
		{
		}
		public void DrawCone(float radius, float height, int upAxis, btTransform^ transform, btVector3^ color)
		{
		}
		public void DrawContactPoint(btVector3^ PointOnB, btVector3^ normalOnB, float distance, int lifeTime, btVector3^ color)
		{
		}
		public void DrawCylinder(float radius, float halfHeight, int upAxis, btTransform^ transform, btVector3^ color)
		{
		}
		public void DrawLine(btVector3^ from, btVector3^ to, btVector3^ color)
		{
		}
		public void DrawLine(btVector3^ from, btVector3^ to, btVector3^ fromColor, btVector3^ toColor)
		{
		}
		public void DrawPlane(btVector3^ planeNormal, float planeConst, btTransform^ transform, btVector3^ color)
		{
		}
		public void DrawSphere(float radius, btTransform^ transform, btVector3^ color)
		{
		}
		public void DrawSphere(btVector3^ p, float radius, btVector3^ color)
		{
		}
		public void DrawSpherePatch(btVector3^ center, btVector3^ up, btVector3^ axis, float radius, float minTh, float maxTh, float minPs, float maxPs, btVector3^ color, float stepDegrees, bool drawCenter)
		{
		}
		public void DrawTransform(btTransform^ transform, float orthoLen)
		{
		}
		public void DrawTriangle(btVector3^ v0, btVector3^ v1, btVector3^ v2, btVector3^ , btVector3^ , btVector3^ , btVector3^ color, float alpha)
		{
		}
		public void DrawTriangle(btVector3^ v0, btVector3^ v1, btVector3^ v2, btVector3^ color, float )
		{
		}
		public void FlushLines()
		{
		}
		public void GetDebugMode()
		{
		}
		public void GetDefaultColors()
		{
		}
		public void ReportErrorWarning(char^ warningString)
		{
		}
		public void SetDebugMode(int debugMode)
		{
		}
		public void SetDefaultColors(DefaultColors^ )
		{
		}
	public class DefaultColors
	{
		public DefaultColors()
		{
		}
	}
	public enum DebugDrawModes
	{
		NoDebug = 0,
		DrawWireframe = 1,
		DrawAabb = 2,
		DrawFeaturesText = 4,
		DrawContactPoints = 8,
		NoDeactivation = 16,
		NoHelpText = 32,
		DrawText = 64,
		ProfileTimings = 128,
		EnableSatComparison = 256,
		DisableBulletLCP = 512,
		EnableCCD = 1024,
		DrawConstraints = (1<<11),
		DrawConstraintLimits = (1<<12),
		FastWireframe = (1<<13),
		DrawNormals = (1<<14),
		DrawFrames = (1<<15),
		MaxDebugDrawMode
	}
	}
}
