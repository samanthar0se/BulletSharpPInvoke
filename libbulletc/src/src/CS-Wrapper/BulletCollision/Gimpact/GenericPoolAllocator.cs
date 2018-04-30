namespace BulletSharp
{
	public class GenericMemoryPool
	{
		public GenericMemoryPool(int element_size, int element_count)
		{
		}
		public void Allocate(int size_bytes)
		{
		}
		public void Allocate_from_free_nodes(int num_elements)
		{
		}
		public void Allocate_from_pool(int num_elements)
		{
		}
		public void End_pool()
		{
		}
		public void FreeMemory(void^ pointer)
		{
		}
		public void Gem_element_size()
		{
		}
		public void Get_allocated_count()
		{
		}
		public void Get_element_data(int element_index)
		{
		}
		public void Get_free_positions_count()
		{
		}
		public void Get_max_element_count()
		{
		}
		public void Get_pool_capacity()
		{
		}
		public void Init_pool(int element_size, int element_count)
		{
		}
	}
	public class GenericPoolAllocator
	{
		public GenericPoolAllocator(int pool_element_size, int pool_element_count)
		{
		}
		public void Allocate(int size_bytes)
		{
		}
		public void Failback_alloc(int size_bytes)
		{
		}
		public void Failback_free(void^ pointer)
		{
		}
		public void FreeMemory(void^ pointer)
		{
		}
		public void Get_pool_capacity()
		{
		}
		public void Push_new_pool()
		{
		}
	}
}
