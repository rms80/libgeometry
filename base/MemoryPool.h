// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#ifndef _RMS_MEMORY_POOL_H
#define _RMS_MEMORY_POOL_H

// ignore annoying warning about dll-interface for vector that is not exposed...
#pragma warning( disable: 4251 )

#include "config.h"
#include <vector>
#include "DynamicVector.h"


namespace rms
{

template<class Type>
class MemoryPool
{
public:
	MemoryPool();
	~MemoryPool();

	Type * Allocate();
	void Free(Type * pData);

	void ClearAll();

protected:
	DynamicVector<Type> m_vStore;

	std::vector<Type *> m_vFree;
};


template<class Type>
MemoryPool<Type>::MemoryPool()
{
}

template<class Type>
MemoryPool<Type>::~MemoryPool()
{
}

template<class Type>
Type * MemoryPool<Type>::Allocate()
{
	if ( m_vFree.empty() )
		return m_vStore.push_back();
	else {
		Type * pType = m_vFree.back();
		m_vFree.pop_back();
		return pType;
	}
}

template<class Type>
void MemoryPool<Type>::Free(Type * pData)
{
	m_vFree.push_back( pData );
}

template<class Type>
void MemoryPool<Type>::ClearAll()
{
	m_vFree.clear();
	m_vStore.clear();
}


} // end namespace rms;


#endif   _RMS_MEMORY_POOL_H