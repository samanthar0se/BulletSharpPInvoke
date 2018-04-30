namespace InverseDynamicsBullet3
{
	public class RigidBody
	{
		public void B3_DECLARE_ALIGNED_ALLOCATOR()
		{
		}
	}
	public class MultiBodyImpl
	{
		public MultiBodyImpl(int num_bodies_, int num_dofs_)
		{
		}
		public void AddRelativeJacobianComponent(RigidBody^ body)
		{
		}
		public void AddUserForce(int body_index, vec3^ body_force)
		{
		}
		public void AddUserMoment(int body_index, vec3^ body_moment)
		{
		}
		public void B3_DECLARE_ALIGNED_ALLOCATOR()
		{
		}
		public void BodyNumDoFs(JointType^ type)
		{
		}
		public void CalculateInverseDynamics(vecx^ q, vecx^ u, vecx^ dot_u, vecx^ joint_forces)
		{
		}
		public void CalculateJacobians(vecx^ q, vecx^ u, KinUpdateType type)
		{
		}
		public void CalculateKinematics(vecx^ q, vecx^ u, vecx^ dot_u, KinUpdateType type)
		{
		}
		public void CalculateMassMatrix(vecx^ q, bool update_kinematics, bool initialize_matrix, bool set_lower_triangular_matrix, matxx^ mass_matrix)
		{
		}
		public void CalculateStaticData()
		{
		}
		public void ClearAllUserForcesAndMoments()
		{
		}
		public void GenerateIndexSets()
		{
		}
		public void GetBodyAngularAcceleration(int body_index, vec3^ world_dot_omega)
		{
		}
		public void GetBodyAngularVelocity(int body_index, vec3^ world_omega)
		{
		}
		public void GetBodyAxisOfMotion(int body_index, vec3^ axis)
		{
		}
		public void GetBodyCoM(int body_index, vec3^ world_com)
		{
		}
		public void GetBodyDotJacobianRotU(int body_index, vec3^ world_dot_jac_rot_u)
		{
		}
		public void GetBodyDotJacobianTransU(int body_index, vec3^ world_dot_jac_trans_u)
		{
		}
		public void GetBodyFirstMassMoment(int body_index, vec3^ first_mass_moment)
		{
		}
		public void GetBodyFrame(int index, vec3^ world_origin, mat33^ body_T_world)
		{
		}
		public void GetBodyJacobianRot(int body_index, mat3x^ world_jac_rot)
		{
		}
		public void GetBodyJacobianTrans(int body_index, mat3x^ world_jac_trans)
		{
		}
		public void GetBodyLinearAcceleration(int body_index, vec3^ world_acceleration)
		{
		}
		public void GetBodyLinearVelocity(int body_index, vec3^ world_velocity)
		{
		}
		public void GetBodyLinearVelocityCoM(int body_index, vec3^ world_velocity)
		{
		}
		public void GetBodyMass(int body_index, int^ mass)
		{
		}
		public void GetBodyOrigin(int body_index, vec3^ world_origin)
		{
		}
		public void GetBodySecondMassMoment(int body_index, mat33^ second_mass_moment)
		{
		}
		public void GetBodyTParentRef(int body_index, mat33^ T)
		{
		}
		public void GetBodyTransform(int body_index, mat33^ world_T_body)
		{
		}
		public void GetDoFOffset(int body_index, int^ q_index)
		{
		}
		public void GetJointType(int body_index, JointType^ joint_type)
		{
		}
		public void GetJointTypeStr(int body_index, char^ joint_type)
		{
		}
		public void GetParentIndex(int body_index, int^ m_parent_index)
		{
		}
		public void GetParentRParentBodyRef(int body_index, vec3^ r)
		{
		}
		public void GetUserInt(int body_index, int^ user_int)
		{
		}
		public void GetUserPtr(int body_index, void^ user_ptr)
		{
		}
		public void JointTypeToString(JointType^ type)
		{
		}
		public void PrintTree()
		{
		}
		public void PrintTree(int index, int indentation)
		{
		}
		public void PrintTreeData()
		{
		}
		public void SetBodyFirstMassMoment(int body_index, vec3^ first_mass_moment)
		{
		}
		public void SetBodyMass(int body_index, int mass)
		{
		}
		public void SetBodySecondMassMoment(int body_index, mat33^ second_mass_moment)
		{
		}
		public void SetGravityInWorldFrame(vec3^ gravity)
		{
		}
		public void SetUserInt(int body_index, int user_int)
		{
		}
		public void SetUserPtr(int body_index, void^ user_ptr)
		{
		}
	public enum KinUpdateType
	{
		Only,
		Velocity,
		VelocityAcceleration
	}
	}
}
