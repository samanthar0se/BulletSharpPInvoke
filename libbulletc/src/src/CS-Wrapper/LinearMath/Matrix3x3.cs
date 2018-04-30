namespace BulletSharp
{
	public class Matrix3x3
	{
		public Matrix3x3()
		{
		}
		public Matrix3x3(btQuaternion^ q)
		{
		}
		public Matrix3x3(float^ xx, float^ xy, float^ xz, float^ yx, float^ yy, float^ yz, float^ zx, float^ zy, float^ zz)
		{
		}
		public Matrix3x3(btMatrix3x3^ other)
		{
		}
		public void Absolute()
		{
		}
		public void Adjoint()
		{
		}
		public void Cofac(int r1, int c1, int r2, int c2)
		{
		}
		public void DeSerialize(btMatrix3x3FloatData^ dataIn)
		{
		}
		public void DeSerializeDouble(btMatrix3x3DoubleData^ dataIn)
		{
		}
		public void DeSerializeFloat(btMatrix3x3FloatData^ dataIn)
		{
		}
		public void Determinant()
		{
		}
		public void Diagonalize(btMatrix3x3^ rot, float tolerance, int maxIter)
		{
		}
		public void ExtractRotation(btQuaternion^ q, float tolerance, int maxIter)
		{
		}
		public void GetColumn(int i)
		{
		}
		public void GetEulerYPR(float^ yaw, float^ pitch, float^ roll)
		{
		}
		public void GetEulerZYX(float^ yaw, float^ pitch, float^ roll, uint solution_number)
		{
		}
		public void GetIdentity()
		{
		}
		public void GetOpenGLSubMatrix(float^ m)
		{
		}
		public void GetRotation(btQuaternion^ q)
		{
		}
		public void GetRow(int i)
		{
		}
		public void Inverse()
		{
		}
		public void Scaled(btVector3^ s)
		{
		}
		public void Serialize(btMatrix3x3FloatData^ dataOut)
		{
		}
		public void SerializeFloat(btMatrix3x3FloatData^ dataOut)
		{
		}
		public void SetEulerYPR(float^ yaw, float^ pitch, float^ roll)
		{
		}
		public void SetEulerZYX(float eulerX, float eulerY, float eulerZ)
		{
		}
		public void SetFromOpenGLSubMatrix(float^ m)
		{
		}
		public void SetIdentity()
		{
		}
		public void SetRotation(btQuaternion^ q)
		{
		}
		public void SetValue(float^ xx, float^ xy, float^ xz, float^ yx, float^ yy, float^ yz, float^ zx, float^ zy, float^ zz)
		{
		}
		public void Solve33(btVector3^ b)
		{
		}
		public void Tdotx(btVector3^ v)
		{
		}
		public void Tdoty(btVector3^ v)
		{
		}
		public void Tdotz(btVector3^ v)
		{
		}
		public void TimesTranspose(btMatrix3x3^ m)
		{
		}
		public void Transpose()
		{
		}
		public void TransposeTimes(btMatrix3x3^ m)
		{
		}
	}
	public class Matrix3x3FloatData
	{
	}
	public class Matrix3x3DoubleData
	{
	}
}
