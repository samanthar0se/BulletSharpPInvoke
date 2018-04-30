namespace BulletSharp
{
	public class b3Chunk
	{
	}
	public class b3Serializer
	{
		public void Allocate(int size, int numElements)
		{
		}
		public void FinalizeChunk(b3Chunk^ chunk, char^ structType, int chunkCode, void^ oldPtr)
		{
		}
		public void FindNameForPointer(void^ ptr)
		{
		}
		public void FindPointer(void^ oldPtr)
		{
		}
		public void FinishSerialization()
		{
		}
		public void GetBufferPointer()
		{
		}
		public void GetCurrentBufferSize()
		{
		}
		public void GetSerializationFlags()
		{
		}
		public void GetUniquePointer(void^ oldPtr)
		{
		}
		public void RegisterNameForPointer(void^ ptr, char^ name)
		{
		}
		public void SerializeName(char^ ptr)
		{
		}
		public void SetSerializationFlags(int flags)
		{
		}
		public void StartSerialization()
		{
		}
	}
	public class b3PointerUid
	{
	}
	public class b3DefaultSerializer
	{
		public B3DefaultSerializer(int totalSize)
		{
		}
		public void Allocate(int size, int numElements)
		{
		}
		public void FinalizeChunk(b3Chunk^ chunk, char^ structType, int chunkCode, void^ oldPtr)
		{
		}
		public void FindNameForPointer(void^ ptr)
		{
		}
		public void FindPointer(void^ oldPtr)
		{
		}
		public void FinishSerialization()
		{
		}
		public void GetBufferPointer()
		{
		}
		public void GetCurrentBufferSize()
		{
		}
		public void GetReverseType(char^ type)
		{
		}
		public void GetSerializationFlags()
		{
		}
		public void GetUniquePointer(void^ oldPtr)
		{
		}
		public void InitDNA(char^ bdnaOrg, int dnalen)
		{
		}
		public void InternalAlloc(int size)
		{
		}
		public void RegisterNameForPointer(void^ ptr, char^ name)
		{
		}
		public void SerializeName(char^ name)
		{
		}
		public void SetSerializationFlags(int flags)
		{
		}
		public void StartSerialization()
		{
		}
		public void WriteDNA()
		{
		}
		public void WriteHeader(byte^ buffer)
		{
		}
	}
}
