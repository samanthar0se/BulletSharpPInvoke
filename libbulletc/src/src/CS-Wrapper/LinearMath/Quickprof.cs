namespace BulletSharp
{
	public class Clock
	{
		public Clock()
		{
		}
		public Clock(btClock^ other)
		{
		}
		public void GetTimeMicroseconds()
		{
		}
		public void GetTimeMilliseconds()
		{
		}
		public void GetTimeNanoseconds()
		{
		}
		public void GetTimeSeconds()
		{
		}
		public void Reset()
		{
		}
	}
	public class CProfileNode
	{
		public CProfileNode(char^ name, CProfileNode^ parent)
		{
		}
		public void Call()
		{
		}
		public void CleanupMemory()
		{
		}
		public void Get_Child()
		{
		}
		public void Get_Name()
		{
		}
		public void Get_Parent()
		{
		}
		public void Get_Sibling()
		{
		}
		public void Get_Sub_Node(char^ name)
		{
		}
		public void Get_Total_Calls()
		{
		}
		public void Get_Total_Time()
		{
		}
		public void GetUserPointer()
		{
		}
		public void Reset()
		{
		}
		public void Return()
		{
		}
		public void SetUserPointer(void^ ptr)
		{
		}
	}
	public class CProfileIterator
	{
		public CProfileIterator(CProfileNode^ start)
		{
		}
		public void Enter_Child(int index)
		{
		}
		public void Enter_Largest_Child()
		{
		}
		public void Enter_Parent()
		{
		}
		public void First()
		{
		}
		public void Get_Current_Name()
		{
		}
		public void Get_Current_Parent_Name()
		{
		}
		public void Get_Current_Parent_Total_Calls()
		{
		}
		public void Get_Current_Parent_Total_Time()
		{
		}
		public void Get_Current_Total_Calls()
		{
		}
		public void Get_Current_Total_Time()
		{
		}
		public void Get_Current_UserPointer()
		{
		}
		public void Is_Done()
		{
		}
		public void Is_Root()
		{
		}
		public void Next()
		{
		}
		public void Set_Current_UserPointer(void^ ptr)
		{
		}
	}
	public class CProfileManager
	{
		public void CleanupMemory()
		{
		}
		public void DumpAll()
		{
		}
		public void DumpRecursive(CProfileIterator^ profileIterator, int spacing)
		{
		}
		public void Get_Frame_Count_Since_Reset()
		{
		}
		public void Get_Iterator()
		{
		}
		public void Get_Time_Since_Reset()
		{
		}
		public void Increment_Frame_Counter()
		{
		}
		public void Release_Iterator(CProfileIterator^ iterator)
		{
		}
		public void Reset()
		{
		}
		public void Start_Profile(char^ name)
		{
		}
		public void Stop_Profile()
		{
		}
	}
	public class CProfileSample
	{
		public CProfileSample(char^ name)
		{
		}
	}
}
