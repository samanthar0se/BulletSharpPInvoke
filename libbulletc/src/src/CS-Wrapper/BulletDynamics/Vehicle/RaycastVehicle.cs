namespace BulletSharp
{
	public class RaycastVehicle
	{
		public RaycastVehicle(btVehicleTuning^ tuning, btRigidBody^ chassis, btVehicleRaycaster^ raycaster)
		{
		}
		public void AddWheel(int^ connectionPointCS0, int^ wheelDirectionCS0, int^ wheelAxleCS, int suspensionRestLength, int wheelRadius, btVehicleTuning^ tuning, bool isFrontWheel)
		{
		}
		public void ApplyEngineForce(int force, int wheel)
		{
		}
		public void DebugDraw(int^ debugDrawer)
		{
		}
		public void DefaultInit(btVehicleTuning^ tuning)
		{
		}
		public void GetChassisWorldTransform()
		{
		}
		public void GetCurrentSpeedKmHour()
		{
		}
		public void GetForwardAxis()
		{
		}
		public void GetForwardVector()
		{
		}
		public void GetNumWheels()
		{
		}
		public void GetRightAxis()
		{
		}
		public void GetRigidBody()
		{
		}
		public void GetRigidBody()
		{
		}
		public void GetSteeringValue(int wheel)
		{
		}
		public void GetUpAxis()
		{
		}
		public void GetUserConstraintId()
		{
		}
		public void GetUserConstraintType()
		{
		}
		public void GetWheelInfo(int index)
		{
		}
		public void GetWheelInfo(int index)
		{
		}
		public void GetWheelTransformWS(int wheelIndex)
		{
		}
		public void RayCast(btWheelInfo^ wheel)
		{
		}
		public void ResetSuspension()
		{
		}
		public void SetBrake(int brake, int wheelIndex)
		{
		}
		public void SetCoordinateSystem(int rightIndex, int upIndex, int forwardIndex)
		{
		}
		public void SetPitchControl(int pitch)
		{
		}
		public void SetSteeringValue(int steering, int wheel)
		{
		}
		public void SetUserConstraintId(int uid)
		{
		}
		public void SetUserConstraintType(int userConstraintType)
		{
		}
		public void UpdateAction(int^ collisionWorld, int step)
		{
		}
		public void UpdateFriction(int timeStep)
		{
		}
		public void UpdateSuspension(int deltaTime)
		{
		}
		public void UpdateVehicle(int step)
		{
		}
		public void UpdateWheelTransform(int wheelIndex, bool interpolatedTransform)
		{
		}
		public void UpdateWheelTransformsWS(btWheelInfo^ wheel, bool interpolatedTransform)
		{
		}
	public class VehicleTuning
	{
		public VehicleTuning()
		{
		}
	}
	}
	public class DefaultVehicleRaycaster
	{
		public DefaultVehicleRaycaster(btDynamicsWorld^ world)
		{
		}
		public void CastRay(int^ from, int^ to, btVehicleRaycasterResult^ result)
		{
		}
	}
}
