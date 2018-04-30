namespace BulletSharp
{
	public class Chunk
	{
	}
	public class Serializer
	{
		public void Allocate(int size, int numElements)
		{
		}
		public void FinalizeChunk(btChunk^ chunk, char^ structType, int chunkCode, void^ oldPtr)
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
		public void GetChunk(int chunkIndex)
		{
		}
		public void GetCurrentBufferSize()
		{
		}
		public void GetNumChunks()
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
	public class PointerUid
	{
	}
	public class BulletSerializedArrays
	{
		public BulletSerializedArrays()
		{
		}
	}
	public class DefaultSerializer
	{
		public DefaultSerializer(int totalSize, byte^ buffer)
		{
		}
		public void Allocate(int size, int numElements)
		{
		}
		public void FinalizeChunk(btChunk^ chunk, char^ structType, int chunkCode, void^ oldPtr)
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
		public void GetChunk(int chunkIndex)
		{
		}
		public void GetCurrentBufferSize()
		{
		}
		public void GetMemoryDna()
		{
		}
		public void GetMemoryDnaSizeInBytes()
		{
		}
		public void GetNumChunks()
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
		public void InsertHeader()
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
