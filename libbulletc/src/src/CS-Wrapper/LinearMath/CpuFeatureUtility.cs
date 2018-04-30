namespace BulletSharp
{
	public class CpuFeatureUtility
	{
		public void GetCpuFeatures()
		{
		}
	[Flags]
	public enum CpuFeature
	{
		None = 0,
		Fma3 = 1,
		Sse41 = 2,
		NeonHpfp = 4
	}
	}
}
