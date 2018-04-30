namespace BulletSharp
{
	public class Quaternion
	{
		public Quaternion()
		{
		}
		public Quaternion(float^ _x, float^ _y, float^ _z, float^ _w)
		{
		}
		public Quaternion(btVector3^ _axis, float^ _angle)
		{
		}
		public Quaternion(float^ yaw, float^ pitch, float^ roll)
		{
		}
		public void Angle(btQuaternion^ q)
		{
		}
		public void AngleShortestPath(btQuaternion^ q)
		{
		}
		public void DeSerialize(btQuaternionFloatData^ dataIn)
		{
		}
		public void DeSerializeDouble(btQuaternionDoubleData^ dataIn)
		{
		}
		public void DeSerializeFloat(btQuaternionFloatData^ dataIn)
		{
		}
		public void Dot(btQuaternion^ q)
		{
		}
		public void Farthest(btQuaternion^ qd)
		{
		}
		public void GetAngle()
		{
		}
		public void GetAngleShortestPath()
		{
		}
		public void GetAxis()
		{
		}
		public void GetEulerZYX(float^ yawZ, float^ pitchY, float^ rollX)
		{
		}
		public void GetIdentity()
		{
		}
		public void GetW()
		{
		}
		public void Inverse()
		{
		}
		public void Length()
		{
		}
		public void Length2()
		{
		}
		public void Nearest(btQuaternion^ qd)
		{
		}
		public void Normalize()
		{
		}
		public void Normalized()
		{
		}
		public void SafeNormalize()
		{
		}
		public void Serialize(btQuaternionFloatData^ dataOut)
		{
		}
		public void SerializeDouble(btQuaternionDoubleData^ dataOut)
		{
		}
		public void SerializeFloat(btQuaternionFloatData^ dataOut)
		{
		}
		public void SetEuler(float^ yaw, float^ pitch, float^ roll)
		{
		}
		public void SetEulerZYX(float^ yawZ, float^ pitchY, float^ rollX)
		{
		}
		public void SetRotation(btVector3^ axis, float^ _angle)
		{
		}
		public void Slerp(btQuaternion^ q, float^ t)
		{
		}
	}
	public class QuaternionFloatData
	{
	}
	public class QuaternionDoubleData
	{
	}
}
