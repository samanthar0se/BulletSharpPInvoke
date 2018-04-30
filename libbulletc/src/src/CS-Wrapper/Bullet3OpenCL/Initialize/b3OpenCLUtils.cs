namespace BulletSharp
{
	public class b3OpenCLPlatformInfo
	{
		public B3OpenCLPlatformInfo()
		{
		}
	}
	public class b3OpenCLUtils
	{
		public void CompileCLKernelFromString(int clContext, int device, char^ kernelSource, char^ kernelName, int^ pErrNum, int prog, char^ additionalMacros)
		{
		}
		public void CompileCLProgramFromString(int clContext, int device, char^ kernelSource, int^ pErrNum, char^ additionalMacros, char^ srcFileNameForCaching, bool disableBinaryCaching)
		{
		}
		public void CreateContextFromPlatform(int platform, int deviceType, int^ pErrNum, void^ pGLCtx, void^ pGLDC, int preferredDeviceIndex, int preferredPlatformIndex)
		{
		}
		public void CreateContextFromType(int deviceType, int^ pErrNum, void^ pGLCtx, void^ pGLDC, int preferredDeviceIndex, int preferredPlatformIndex, int^ platformId)
		{
		}
		public void GetDevice(int cxMainContext, int nr)
		{
		}
		public void GetDeviceInfo(int device, b3OpenCLDeviceInfo^ info)
		{
		}
		public void GetNumDevices(int cxMainContext)
		{
		}
		public void GetNumPlatforms(int^ pErrNum)
		{
		}
		public void GetPlatform(int nr, int^ pErrNum)
		{
		}
		public void GetPlatformInfo(int platform, b3OpenCLPlatformInfo^ platformInfo)
		{
		}
		public void GetSdkVendorName()
		{
		}
		public void PrintDeviceInfo(int device)
		{
		}
		public void PrintPlatformInfo(int platform)
		{
		}
		public void SetCachePath(char^ path)
		{
		}
	}
}
