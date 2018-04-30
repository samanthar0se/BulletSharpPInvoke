namespace BulletSharp
{
	public class GjkEpaSolver2
	{
		public void Distance(int^ shape0, int^ wtrs0, int^ shape1, int^ wtrs1, int^ guess, sResults^ results)
		{
		}
		public void Penetration(int^ shape0, int^ wtrs0, int^ shape1, int^ wtrs1, int^ guess, sResults^ results, bool usemargins)
		{
		}
		public void SignedDistance(int^ position, int margin, int^ shape, int^ wtrs, sResults^ results)
		{
		}
		public void SignedDistance(int^ shape0, int^ wtrs0, int^ shape1, int^ wtrs1, int^ guess, sResults^ results)
		{
		}
		public void StackSizeRequirement()
		{
		}
	public class sResults
	{
	public enum eStatus
	{
		Separated,
		Penetrating,
		GjkFailed,
		EpaFailed
	}
	}
	}
}
