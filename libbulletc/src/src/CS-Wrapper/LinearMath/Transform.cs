namespace BulletSharp
{
	public class Transform
	{
		public Transform()
		{
		}
		public Transform(btQuaternion^ q, btVector3^ c)
		{
		}
		public Transform(btMatrix3x3^ b, btVector3^ c)
		{
		}
		public Transform(btTransform^ other)
		{
		}
		public void DeSerialize(btTransformFloatData^ dataIn)
		{
		}
		public void DeSerializeDouble(btTransformDoubleData^ dataIn)
		{
		}
		public void DeSerializeFloat(btTransformFloatData^ dataIn)
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
		public void InverseTimes(btTransform^ t)
		{
		}
		public void InvXform(btVector3^ inVec)
		{
		}
		public void Mult(btTransform^ t1, btTransform^ t2)
		{
		}
		public void Serialize(btTransformFloatData^ dataOut)
		{
		}
		public void SerializeFloat(btTransformFloatData^ dataOut)
		{
		}
		public void SetBasis(btMatrix3x3^ basis)
		{
		}
		public void SetFromOpenGLMatrix(float^ m)
		{
		}
		public void SetIdentity()
		{
		}
		public void SetOrigin(btVector3^ origin)
		{
		}
		public void SetRotation(btQuaternion^ q)
		{
		}
	}
	public class TransformFloatData
	{
	}
	public class TransformDoubleData
	{
	}
}
