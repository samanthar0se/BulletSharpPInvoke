namespace BulletSharp
{
	public class b3Matrix3x3
	{
		public B3Matrix3x3()
		{
		}
		public B3Matrix3x3(b3Quaternion^ q)
		{
		}
		public B3Matrix3x3(float^ xx, float^ xy, float^ xz, float^ yx, float^ yy, float^ yz, float^ zx, float^ zy, float^ zz)
		{
		}
		public B3Matrix3x3(b3Matrix3x3^ other)
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
		public void DeSerialize(b3Matrix3x3FloatData^ dataIn)
		{
		}
		public void DeSerializeDouble(b3Matrix3x3DoubleData^ dataIn)
		{
		}
		public void DeSerializeFloat(b3Matrix3x3FloatData^ dataIn)
		{
		}
		public void Determinant()
		{
		}
		public void Diagonalize(b3Matrix3x3^ rot, float threshold, int maxSteps)
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
		public void GetRotation(b3Quaternion^ q)
		{
		}
		public void GetRow(int i)
		{
		}
		public void Inverse()
		{
		}
		public void Scaled(b3Vector3^ s)
		{
		}
		public void Serialize(b3Matrix3x3FloatData^ dataOut)
		{
		}
		public void SerializeFloat(b3Matrix3x3FloatData^ dataOut)
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
		public void SetRotation(b3Quaternion^ q)
		{
		}
		public void SetValue(float^ xx, float^ xy, float^ xz, float^ yx, float^ yy, float^ yz, float^ zx, float^ zy, float^ zz)
		{
		}
		public void Tdotx(b3Vector3^ v)
		{
		}
		public void Tdoty(b3Vector3^ v)
		{
		}
		public void Tdotz(b3Vector3^ v)
		{
		}
		public void TimesTranspose(b3Matrix3x3^ m)
		{
		}
		public void Transpose()
		{
		}
		public void TransposeTimes(b3Matrix3x3^ m)
		{
		}
	}
	public class b3Matrix3x3FloatData
	{
	}
	public class b3Matrix3x3DoubleData
	{
	}
}
