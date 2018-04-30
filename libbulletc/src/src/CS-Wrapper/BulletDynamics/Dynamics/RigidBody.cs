namespace BulletSharp
{
	public class RigidBody
	{
		public RigidBody(btRigidBodyConstructionInfo^ constructionInfo)
		{
		}
		public RigidBody(int mass, btMotionState^ motionState, btCollisionShape^ collisionShape, int^ localInertia)
		{
		}
		public void AddConstraintRef(btTypedConstraint^ c)
		{
		}
		public void ApplyCentralForce(int^ force)
		{
		}
		public void ApplyCentralImpulse(int^ impulse)
		{
		}
		public void ApplyDamping(int timeStep)
		{
		}
		public void ApplyForce(int^ force, int^ rel_pos)
		{
		}
		public void ApplyGravity()
		{
		}
		public void ApplyImpulse(int^ impulse, int^ rel_pos)
		{
		}
		public void ApplyTorque(int^ torque)
		{
		}
		public void ApplyTorqueImpulse(int^ torque)
		{
		}
		public void ATTRIBUTE_ALIGNED16(int m_deltaLinearVelocity)
		{
		}
		public void CalculateSerializeBufferSize()
		{
		}
		public void ClearForces()
		{
		}
		public void ComputeGyroscopicForceExplicit(int maxGyroscopicForce)
		{
		}
		public void ComputeGyroscopicImpulseImplicit_Body(int step)
		{
		}
		public void ComputeGyroscopicImpulseImplicit_World(int dt)
		{
		}
		public void GetAabb(int^ aabbMin, int^ aabbMax)
		{
		}
		public void GetAngularDamping()
		{
		}
		public void GetAngularFactor()
		{
		}
		public void GetAngularSleepingThreshold()
		{
		}
		public void GetAngularVelocity()
		{
		}
		public void GetCenterOfMassPosition()
		{
		}
		public void GetCenterOfMassTransform()
		{
		}
		public void GetConstraintRef(int index)
		{
		}
		public void GetFlags()
		{
		}
		public void GetGravity()
		{
		}
		public void GetInvInertiaDiagLocal()
		{
		}
		public void GetInvInertiaTensorWorld()
		{
		}
		public void GetInvMass()
		{
		}
		public void GetLinearDamping()
		{
		}
		public void GetLinearFactor()
		{
		}
		public void GetLinearSleepingThreshold()
		{
		}
		public void GetLinearVelocity()
		{
		}
		public void GetLocalInertia()
		{
		}
		public void GetNumConstraintRefs()
		{
		}
		public void GetOrientation()
		{
		}
		public void GetTotalForce()
		{
		}
		public void GetTotalTorque()
		{
		}
		public void GetVelocityInLocalPoint(int^ rel_pos)
		{
		}
		public void IntegrateVelocities(int step)
		{
		}
		public void IsInWorld()
		{
		}
		public void PredictIntegratedTransform(int step, int^ predictedTransform)
		{
		}
		public void ProceedToTransform(int^ newTrans)
		{
		}
		public void RemoveConstraintRef(btTypedConstraint^ c)
		{
		}
		public void SaveKinematicState(int step)
		{
		}
		public void Serialize(void^ dataBuffer, btSerializer^ serializer)
		{
		}
		public void SerializeSingleObject(btSerializer^ serializer)
		{
		}
		public void SetAngularFactor(int^ angFac)
		{
		}
		public void SetAngularFactor(int angFac)
		{
		}
		public void SetAngularVelocity(int^ ang_vel)
		{
		}
		public void SetCenterOfMassTransform(int^ xform)
		{
		}
		public void SetDamping(int lin_damping, int ang_damping)
		{
		}
		public void SetFlags(int flags)
		{
		}
		public void SetGravity(int^ acceleration)
		{
		}
		public void SetInvInertiaDiagLocal(int^ diagInvInertia)
		{
		}
		public void SetLinearFactor(int^ linearFactor)
		{
		}
		public void SetLinearVelocity(int^ lin_vel)
		{
		}
		public void SetSleepingThresholds(int linear, int angular)
		{
		}
		public void SetupRigidBody(btRigidBodyConstructionInfo^ constructionInfo)
		{
		}
		public void Translate(int^ v)
		{
		}
		public void Upcast(int^ colObj)
		{
		}
		public void Upcast(int^ colObj)
		{
		}
		public void UpdateInertiaTensor()
		{
		}
	public class RigidBodyConstructionInfo
	{
		public RigidBodyConstructionInfo(int mass, btMotionState^ motionState, btCollisionShape^ collisionShape, int^ localInertia)
		{
		}
	}
	}
	public class RigidBodyFloatData
	{
	}
	public class RigidBodyDoubleData
	{
	}
}
