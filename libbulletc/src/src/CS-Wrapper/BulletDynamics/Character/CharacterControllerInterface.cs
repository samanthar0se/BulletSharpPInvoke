namespace BulletSharp
{
	public class CharacterControllerInterface
	{
		public CharacterControllerInterface()
		{
		}
		public void CanJump()
		{
		}
		public void Jump(int^ dir)
		{
		}
		public void OnGround()
		{
		}
		public void PlayerStep(btCollisionWorld^ collisionWorld, int dt)
		{
		}
		public void PreStep(btCollisionWorld^ collisionWorld)
		{
		}
		public void Reset(btCollisionWorld^ collisionWorld)
		{
		}
		public void SetUpInterpolate(bool value)
		{
		}
		public void SetVelocityForTimeInterval(int^ velocity, int timeInterval)
		{
		}
		public void SetWalkDirection(int^ walkDirection)
		{
		}
		public void Warp(int^ origin)
		{
		}
	}
}
