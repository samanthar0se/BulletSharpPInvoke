namespace bParse
{
	public class bFile
	{
		public BFile(char^ filename, char^ headerString)
		{
		}
		public BFile(char^ memoryBuffer, int len, char^ headerString)
		{
		}
		public void AddDataBlock(char^ dataBlock)
		{
		}
		public void DumpChunks(bDNA^ dna)
		{
		}
		public void FindLibPointer(void^ ptr)
		{
		}
		public void GetAsString(int code)
		{
		}
		public void GetFileDNA()
		{
		}
		public void GetFileElement(short^ firstStruct, char^ lookupName, char^ lookupType, char^ data, short^ foundPos)
		{
		}
		public void GetFlags()
		{
		}
		public void GetLibPointers()
		{
		}
		public void GetMatchingFileDNA(short^ old, char^ lookupName, char^ lookupType, char^ strcData, char^ data, bool fixupPointers)
		{
		}
		public void GetNextBlock(bChunkInd^ dataChunk, char^ dataPtr, int flags)
		{
		}
		public void GetVersion()
		{
		}
		public void Ok()
		{
		}
		public void Parse(int verboseMode)
		{
		}
		public void ParseData()
		{
		}
		public void ParseHeader()
		{
		}
		public void ParseInternal(int verboseMode, char^ memDna, int memDnaLength)
		{
		}
		public void ParseStruct(char^ strcPtr, char^ dtPtr, int old_dna, int new_dna, bool fixupPointers)
		{
		}
		public void PreSwap()
		{
		}
		public void ReadStruct(char^ head, bChunkInd^ chunk)
		{
		}
		public void ResolvePointers(int verboseMode)
		{
		}
		public void ResolvePointersChunk(bChunkInd^ dataChunk, int verboseMode)
		{
		}
		public void ResolvePointersMismatch()
		{
		}
		public void ResolvePointersStructRecursive(char^ strcPtr, int old_dna, int verboseMode, int recursion)
		{
		}
		public void SafeSwapPtr(char^ dst, char^ src)
		{
		}
		public void Swap(char^ head, bChunkInd^ ch, bool ignoreEndianFlag)
		{
		}
		public void SwapData(char^ data, short type, int arraySize, bool ignoreEndianFlag)
		{
		}
		public void SwapDNA(char^ ptr)
		{
		}
		public void SwapLen(char^ dataPtr)
		{
		}
		public void SwapStruct(int dna_nr, char^ data, bool ignoreEndianFlag)
		{
		}
		public void UpdateOldPointers()
		{
		}
		public void Write(char^ fileName, bool fixupPointers)
		{
		}
		public void WriteChunks(int^ fp, bool fixupPointers)
		{
		}
		public void WriteDNA(int^ fp)
		{
		}
		public void WriteFile(char^ fileName)
		{
		}
	}
}
