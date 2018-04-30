namespace BulletSharp
{
	public class GjkEpaSolver3
	{
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
	public class MinkowskiDiff
	{
		public MinkowskiDiff<btConvexTemplate>([unexposed type]^ a, [unexposed type]^ b)
		{
		}
		public void EnableMargin(bool enable)
		{
		}
		public void Support(int^ d)
		{
		}
		public void Support(int^ d, uint index)
		{
		}
		public void Support0(int^ d)
		{
		}
		public void Support1(int^ d)
		{
		}
	}
	public class GJK
	{
		public GJK<btConvexTemplate>([unexposed type]^ a, [unexposed type]^ b)
		{
		}
		public void Appendvertice(sSimplex^ simplex, int^ v)
		{
		}
		public void Det(int^ a, int^ b, int^ c)
		{
		}
		public void EncloseOrigin()
		{
		}
		public void Evaluate(MinkowskiDiff^ shapearg, int^ guess)
		{
		}
		public void Getsupport(int^ d, sSV^ sv)
		{
		}
		public void Initialize()
		{
		}
		public void Projectorigin(int^ a, int^ b, int^ w, uint^ m)
		{
		}
		public void Projectorigin(int^ a, int^ b, int^ c, int^ w, uint^ m)
		{
		}
		public void Projectorigin(int^ a, int^ b, int^ c, int^ d, int^ w, uint^ m)
		{
		}
		public void Removevertice(sSimplex^ simplex)
		{
		}
	public class sSimplex
	{
	}
	public class sSV
	{
	}
	}
	public class EPA
	{
		public EPA<btConvexTemplate>()
		{
		}
		public void Append(sList^ list, sFace^ face)
		{
		}
		public void Bind(sFace^ fa, uint ea, sFace^ fb, uint eb)
		{
		}
		public void Evaluate(GJK^ gjk, int^ guess)
		{
		}
		public void Expand(uint pass, [unexposed type]^ w, sFace^ f, uint e, sHorizon^ horizon)
		{
		}
		public void Findbest()
		{
		}
		public void Getedgedist(sFace^ face, [unexposed type]^ a, [unexposed type]^ b, int^ dist)
		{
		}
		public void Initialize()
		{
		}
		public void Newface([unexposed type]^ a, [unexposed type]^ b, [unexposed type]^ c, bool forced)
		{
		}
		public void Remove(sList^ list, sFace^ face)
		{
		}
	public class sFace
	{
	}
	public class sHorizon
	{
		public SHorizon()
		{
		}
	}
	public class sList
	{
		public SList()
		{
		}
	}
	}
}
