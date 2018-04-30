namespace BulletSharp
{
	public class b3Transform
	{
		public B3Transform()
		{
		}
		public B3Transform(b3Quaternion^ q, b3Vector3^ c)
		{
		}
		public B3Transform(b3Matrix3x3^ b, b3Vector3^ c)
		{
		}
		public B3Transform(b3Transform^ other)
		{
		}
		public void DeSerialize(b3TransformFloatData^ dataIn)
		{
		}
		public void DeSerializeDouble(b3TransformDoubleData^ dataIn)
		{
		}
		public void DeSerializeFloat(b3TransformFloatData^ dataIn)
		{
		}
		public void GetBasis()
		{
		}
		public void GetBasis()
		{
		}
		public void GetIdentity()
		{
		}
		public void GetOpenGLMatrix(float^ m)
		{
		}
		public void GetOrigin()
		{
		}
		public void GetOrigin()
		{
		}
		public void GetRotation()
		{
		}
		public void Inverse()
		{
		}
		public void InverseTimes(b3Transform^ t)
		{
		}
		public void InvXform(b3Vector3^ inVec)
		{
		}
		public void Mult(b3Transform^ t1, b3Transform^ t2)
		{
		}
		public void Serialize(b3TransformFloatData^ dataOut)
		{
		}
		public void SerializeFloat(b3TransformFloatData^ dataOut)
		{
		}
		public void SetBasis(b3Matrix3x3^ basis)
		{
		}
		public void SetFromOpenGLMatrix(float^ m)
		{
		}
		public void SetIdentity()
		{
		}
		public void SetOrigin(b3Vector3^ origin)
		{
		}
		public void SetRotation(b3Quaternion^ q)
		{
		}
	}
	public class b3TransformFloatData
	{
	}
	public class b3TransformDoubleData
	{
	}
}
