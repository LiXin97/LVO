/// \author Jonas Witt
/// \file
/// \brief Implements an object pool template class for fast allocation.


#pragma once
#include <vector>
#include <algorithm>


namespace lvl {


template<typename ElementType>
class ObjectPool
{
public:
	ObjectPool() {}

	ObjectPool(unsigned int size)
	{
		reserve(size);
	}

	~ObjectPool()
	{
		release();
	}

	void release()
	{
		freeAll();

		for(unsigned int i=0; i < m_freeMemory.size(); ++i)
		{
			delete m_freeMemory[i];
		}

		m_freeMemory.clear();
	}

	void reserve(size_t size)
	{	
		if(size > m_freeMemory.capacity())
		{
			//release();
			grow(size - m_freeMemory.capacity());
		}
	}

	
	ElementType* malloc()
	{
		if(m_freeMemory.empty())
			grow(::std::max((size_t)10U, m_freeMemory.capacity())); // double capacity

		ElementType* ret = m_freeMemory.back();
		m_allocatedMemory.push_back(ret);
		m_freeMemory.pop_back();
		return ret;
	}
    /*void free(ElementType* p)
	{
		for(unsigned int i=0; i
	}*/
	void freeAll()
	{
		m_freeMemory.insert(m_freeMemory.end(), m_allocatedMemory.begin(), m_allocatedMemory.end());
		m_allocatedMemory.clear();
	}
    //bool is_from(ElementType* p) const;

protected:
	void grow(size_t bySize)
	{
		size_t oldSize = m_freeMemory.size();

		m_freeMemory.reserve(oldSize + bySize);
		m_allocatedMemory.reserve(oldSize + bySize);

		for(size_t i=oldSize; i < oldSize + bySize; ++i)
		{
			m_freeMemory.push_back(new ElementType());
		}
	}
	::std::vector<ElementType*> m_freeMemory;
	::std::vector<ElementType*> m_allocatedMemory;

};


} // namespace lvl
