namespace InverseDynamicsBullet3
{
	public class InertiaData
	{
		public void B3_DECLARE_ALIGNED_ALLOCATOR()
		{
		}
	}
	public class JointData
	{
		public void B3_DECLARE_ALIGNED_ALLOCATOR()
		{
		}
	}
	public class InitCache
	{
		public InitCache()
		{
		}
		public void AddBody(int body_index, int parent_index, JointType joint_type, vec3^ parent_r_parent_body_ref, mat33^ body_T_parent_ref, vec3^ body_axis_of_motion, int mass, vec3^ body_r_body_com, mat33^ body_I_body, int user_int, void^ user_ptr)
		{
		}
		public void B3_DECLARE_ALIGNED_ALLOCATOR()
		{
		}
		public void BuildIndexSets()
		{
		}
		public void GetInertiaData(int index, InertiaData^ inertia)
		{
		}
		public void GetJointData(int index, JointData^ joint)
		{
		}
		public void GetParentIndexArray(int^ parent_index)
		{
		}
		public void GetUserInt(int index, int^ user_int)
		{
		}
		public void GetUserPtr(int index, void^ user_ptr)
		{
		}
		public void NumBodies()
		{
		}
		public void NumDoFs()
		{
		}
	}
}
