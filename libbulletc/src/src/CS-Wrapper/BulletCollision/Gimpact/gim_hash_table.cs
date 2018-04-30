namespace BulletSharp
{
	public class GIM_HASH_TABLE_NODE
	{
		public GIM_HASH_TABLE_NODE<T>()
		{
		}
		public GIM_HASH_TABLE_NODE<T>(GIM_HASH_TABLE_NODE^ value)
		{
		}
		public GIM_HASH_TABLE_NODE<T>(uint key, [unexposed type]^ data)
		{
		}
	}
	public class GIM_HASH_NODE_GET_KEY
	{
	}
	public class GIM_HASH_NODE_CMP_KEY_MACRO
	{
	}
	public class GIM_HASH_NODE_CMP_MACRO
	{
	}
	public class gim_hash_table
	{
		public Gim_hash_table<T>(uint reserve_size, uint node_size, uint min_hash_table_size)
		{
		}
		public void _assign_hash_table_cell(uint hashkey)
		{
		}
		public void _clear_table_memory()
		{
		}
		public void _destroy()
		{
		}
		public void _erase_by_index_hash_table(uint index)
		{
		}
		public void _erase_hash_table(uint hashkey)
		{
		}
		public void _erase_sorted(uint index)
		{
		}
		public void _erase_unsorted(uint index)
		{
		}
		public void _find_avaliable_cell(uint hashkey)
		{
		}
		public void _find_cell(uint hashkey)
		{
		}
		public void _insert_hash_table(uint hashkey, [unexposed type]^ value)
		{
		}
		public void _insert_hash_table_replace(uint hashkey, [unexposed type]^ value)
		{
		}
		public void _insert_in_pos(uint hashkey, [unexposed type]^ value, uint pos)
		{
		}
		public void _insert_sorted(uint hashkey, [unexposed type]^ value)
		{
		}
		public void _insert_sorted_replace(uint hashkey, [unexposed type]^ value)
		{
		}
		public void _insert_unsorted(uint hashkey, [unexposed type]^ value)
		{
		}
		public void _invalidate_keys()
		{
		}
		public void _rehash()
		{
		}
		public void _reserve_table_memory(uint newtablesize)
		{
		}
		public void _resize_table(uint newsize)
		{
		}
		public void Check_for_switching_to_hashtable()
		{
		}
		public void Clear()
		{
		}
		public void Erase_by_index(uint index)
		{
		}
		public void Erase_by_index_unsorted(uint index)
		{
		}
		public void Erase_by_key(uint hashkey)
		{
		}
		public void Find(uint hashkey)
		{
		}
		public void Get_key(uint index)
		{
		}
		public void Get_value(uint hashkey)
		{
		}
		public void Get_value_by_index(uint index)
		{
		}
		public void Insert(uint hashkey, [unexposed type]^ element)
		{
		}
		public void Insert_override(uint hashkey, [unexposed type]^ element)
		{
		}
		public void Insert_unsorted(uint hashkey, [unexposed type]^ element)
		{
		}
		public void Is_hash_table()
		{
		}
		public void Is_sorted()
		{
		}
		public void Set_sorted(bool value)
		{
		}
		public void Size()
		{
		}
		public void Sort()
		{
		}
		public void Switch_to_hashtable()
		{
		}
		public void Switch_to_sorted_array()
		{
		}
	}
}
