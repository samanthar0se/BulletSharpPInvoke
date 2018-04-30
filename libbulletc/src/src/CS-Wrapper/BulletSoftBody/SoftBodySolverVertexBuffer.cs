namespace BulletSharp
{
	public class VertexBufferDescriptor
	{
		public VertexBufferDescriptor()
		{
		}
		public void GetBufferType()
		{
		}
		public void GetNormalOffset()
		{
		}
		public void GetNormalStride()
		{
		}
		public void GetVertexOffset()
		{
		}
		public void GetVertexStride()
		{
		}
		public void HasNormals()
		{
		}
		public void HasVertexPositions()
		{
		}
	public enum BufferTypes
	{
		CpuBuffer,
		Dx11Buffer,
		OpenglBuffer
	}
	}
	public class CPUVertexBufferDescriptor
	{
		public CPUVertexBufferDescriptor(float^ basePointer, int vertexOffset, int vertexStride)
		{
		}
		public CPUVertexBufferDescriptor(float^ basePointer, int vertexOffset, int vertexStride, int normalOffset, int normalStride)
		{
		}
		public void GetBasePointer()
		{
		}
		public void GetBufferType()
		{
		}
	}
}
