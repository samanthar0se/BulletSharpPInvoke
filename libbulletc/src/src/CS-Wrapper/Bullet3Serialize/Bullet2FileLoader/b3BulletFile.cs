namespace bParse
{
	public class b3BulletFile
	{
		public B3BulletFile()
		{
		}
		public B3BulletFile(char^ fileName)
		{
		}
		public B3BulletFile(char^ memoryBuffer, int len)
		{
		}
		public void AddDataBlock(char^ dataBlock)
		{
		}
		public void AddStruct(char^ structType, void^ data, int len, void^ oldPtr, int code)
		{
		}
		public void Parse(int verboseMode)
		{
		}
		public void ParseData()
		{
		}
		public void Write(char^ fileName, bool fixupPointers)
		{
		}
		public void WriteDNA(int^ fp)
		{
		}
	}
}
