namespace bParse
{
	public class bNameInfo
	{
	}
	public class bDNA
	{
		public BDNA()
		{
		}
		public void DumpTypeDefinitions()
		{
		}
		public void FlagEqual(int dna_nr)
		{
		}
		public void FlagNone(int dna_nr)
		{
		}
		public void FlagNotEqual(int dna_nr)
		{
		}
		public void GetArraySize(char^ str)
		{
		}
		public void GetArraySizeNew(short name)
		{
		}
		public void GetElementSize(short type, short name)
		{
		}
		public void GetLength(int ind)
		{
		}
		public void GetName(int ind)
		{
		}
		public void GetNumNames()
		{
		}
		public void GetNumStructs()
		{
		}
		public void GetPointerSize()
		{
		}
		public void GetReverseType(short type)
		{
		}
		public void GetReverseType(char^ type)
		{
		}
		public void GetStruct(int ind)
		{
		}
		public void GetType(int ind)
		{
		}
		public void Init(char^ data, int len, bool swap)
		{
		}
		public void InitCmpFlags(bDNA^ memDNA)
		{
		}
		public void InitRecurseCmpFlags(int i)
		{
		}
		public void LessThan(bDNA^ other)
		{
		}
	[Flags]
	public enum FileDNAFlags
	{
		None = 0,
		StructNequ,
		StructEqu
	}
	}
}
