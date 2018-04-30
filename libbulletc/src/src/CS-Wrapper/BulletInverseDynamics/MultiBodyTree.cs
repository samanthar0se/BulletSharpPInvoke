namespace InverseDynamicsBullet3
{
	public class MultiBodyTree
	{
		public MultiBodyTree()
		{
		}
		public void AddBody(int body_index, int parent_index, JointType joint_type, vec3^ parent_r_parent_body_ref, mat33^ body_T_parent_ref, vec3^ body_axis_of_motion, int mass, vec3^ body_r_body_com, mat33^ body_I_body, int user_int, void^ user_ptr)
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
		public void CalculateInverseDynamics(vecx^ q, vecx^ u, vecx^ dot_u, vecx^ joint_forces)
		{
		}
		public void CalculateJacobians(vecx^ q, vecx^ u)
		{
		}
		public void CalculateJacobians(vecx^ q)
		{
		}
		public void CalculateKinematics(vecx^ q, vecx^ u, vecx^ dot_u)
		{
		}
		public void CalculateMassMatrix(vecx^ q, bool update_kinematics, bool initialize_matrix, bool set_lower_triangular_matrix, matxx^ mass_matrix)
		{
		}
		public void CalculateMassMatrix(vecx^ q, matxx^ mass_matrix)
		{
		}
		public void CalculatePositionAndVelocityKinematics(vecx^ q, vecx^ u)
		{
		}
		public void CalculatePositionKinematics(vecx^ q)
		{
		}
		public void ClearAllUserForcesAndMoments()
		{
		}
		public void Finalize()
		{
		}
		public void GetAcceptInvalidMassProperties()
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
		public void GetDoFOffset(int body_index, int^ q_offset)
		{
		}
		public void GetJointType(int body_index, JointType^ joint_type)
		{
		}
		public void GetJointTypeStr(int body_index, char^ joint_type)
		{
		}
		public void GetParentIndex(int body_index, int^ parent_index)
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
		public void NumBodies()
		{
		}
		public void NumDoFs()
		{
		}
		public void PrintTree()
		{
		}
		public void PrintTreeData()
		{
		}
		public void SetAcceptInvalidMassParameters(bool flag)
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
	}
}
