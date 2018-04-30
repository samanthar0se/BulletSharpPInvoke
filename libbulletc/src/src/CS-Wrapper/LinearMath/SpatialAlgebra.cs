namespace BulletSharp
{
	public class SpatialForceVector
	{
		public SpatialForceVector()
		{
		}
		public SpatialForceVector(btVector3^ angular, btVector3^ linear)
		{
		}
		public SpatialForceVector(float^ ax, float^ ay, float^ az, float^ lx, float^ ly, float^ lz)
		{
		}
		public void AddAngular(btVector3^ angular)
		{
		}
		public void AddLinear(btVector3^ linear)
		{
		}
		public void AddValue(float^ ax, float^ ay, float^ az, float^ lx, float^ ly, float^ lz)
		{
		}
		public void AddVector(btVector3^ angular, btVector3^ linear)
		{
		}
		public void GetAngular()
		{
		}
		public void GetLinear()
		{
		}
		public void SetAngular(btVector3^ angular)
		{
		}
		public void SetLinear(btVector3^ linear)
		{
		}
		public void SetValue(float^ ax, float^ ay, float^ az, float^ lx, float^ ly, float^ lz)
		{
		}
		public void SetVector(btVector3^ angular, btVector3^ linear)
		{
		}
		public void SetZero()
		{
		}
	}
	public class SpatialMotionVector
	{
		public SpatialMotionVector()
		{
		}
		public SpatialMotionVector(btVector3^ angular, btVector3^ linear)
		{
		}
		public void AddAngular(btVector3^ angular)
		{
		}
		public void AddLinear(btVector3^ linear)
		{
		}
		public void AddValue(float^ ax, float^ ay, float^ az, float^ lx, float^ ly, float^ lz)
		{
		}
		public void AddVector(btVector3^ angular, btVector3^ linear)
		{
		}
		public void Dot(btSpatialForceVector^ b)
		{
		}
		public void GetAngular()
		{
		}
		public void GetLinear()
		{
		}
		public void SetAngular(btVector3^ angular)
		{
		}
		public void SetLinear(btVector3^ linear)
		{
		}
		public void SetValue(float^ ax, float^ ay, float^ az, float^ lx, float^ ly, float^ lz)
		{
		}
		public void SetVector(btVector3^ angular, btVector3^ linear)
		{
		}
		public void SetZero()
		{
		}
	}
	public class SymmetricSpatialDyad
	{
		public SymmetricSpatialDyad()
		{
		}
		public SymmetricSpatialDyad(btMatrix3x3^ topLeftMat, btMatrix3x3^ topRightMat, btMatrix3x3^ bottomLeftMat)
		{
		}
		public void AddMatrix(btMatrix3x3^ topLeftMat, btMatrix3x3^ topRightMat, btMatrix3x3^ bottomLeftMat)
		{
		}
		public void SetIdentity()
		{
		}
		public void SetMatrix(btMatrix3x3^ topLeftMat, btMatrix3x3^ topRightMat, btMatrix3x3^ bottomLeftMat)
		{
		}
	}
	public class SpatialTransformationMatrix
	{
		public void TransformInverse(btSymmetricSpatialDyad^ inMat, btSymmetricSpatialDyad^ outMat, eOutputOperation outOp)
		{
		}
	[Flags]
	public enum eOutputOperation
	{
		None = 0,
		Add = 1,
		Subtract = 2
	}
	}
}
