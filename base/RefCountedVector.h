// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#pragma once

// ignore annoying warning about dll-interface for vector that is not exposed...
#pragma warning( push )
#pragma warning( disable: 4251 )

#include "config.h"
#include <vector>
#include <string>
#include <strstream>

#include "MemoryPool.h"

namespace rms {

#define INVALID_REFCOUNT std::numeric_limits<int>::max()

template<class Type>
class RefCountedVector
{
public:
	RefCountedVector() 	
		{ clear(); }
	virtual ~RefCountedVector()
		{}

	inline bool isValid( int nIndex ) const {
		return ( nIndex < (int)m_vData.size() && m_vData[nIndex].nRefCount > 0 && m_vData[nIndex].nRefCount != INVALID_REFCOUNT );
	}

	inline int refCount( int nIndex ) const {
		lgASSERT( isValid(nIndex)  );
		return ( m_vData[nIndex].nRefCount > 0 && m_vData[nIndex].nRefCount != INVALID_REFCOUNT )
			? m_vData[nIndex].nRefCount : 0;
	}

	inline int increment( int nIndex ) {
		lgASSERT( isValid(nIndex)  );
		return ++m_vData[nIndex].nRefCount;
	}

	inline void decrement( int nIndex ) {
		lgASSERT( isValid(nIndex) );
		--m_vData[nIndex].nRefCount;
		lgASSERT( m_vData[nIndex].nRefCount >= 0 );
		if ( m_vData[nIndex].nRefCount == 0 ) {			// add to empty list
			if ( m_nFirstFree == INVALID_REFCOUNT ) {
				m_vData[nIndex].nRefCount = INVALID_REFCOUNT;
				m_nFirstFree = nIndex;
			} else {
				m_vData[nIndex].nRefCount = -(m_nFirstFree+1);
				m_nFirstFree = nIndex;
			}
			m_nUsedCount--;
		}
	}

	inline int insert( const Type & t ) {
		m_nUsedCount++;
		if ( m_nFirstFree == INVALID_REFCOUNT ) {
			RefEntry r;
			r.nRefCount = 1;
			r.vecData = t;
			m_vData.push_back(r);
			return (int)(m_vData.size() - 1);
		} else {
			int nFree = m_nFirstFree;
			lgASSERT(nFree >= 0 && nFree < (int)m_vData.size());
			int nNextFree = m_vData[ nFree ].nRefCount;
			m_vData[nFree].nRefCount = 1;
			m_vData[nFree].vecData = t;
			if ( nNextFree < 0 )
				m_nFirstFree = -(nNextFree+1);
			else {
				lgASSERT( nNextFree == INVALID_REFCOUNT );
				m_nFirstFree = nNextFree;
			}
			return nFree;
		}
	}

	inline void remove( int nIndex ) {		// force remove
		lgASSERT( (unsigned int)nIndex < m_vData.size() );
		if ( m_vData[nIndex].nRefCount < 0 || m_vData[nIndex].nRefCount == INVALID_REFCOUNT )
			return;			// already removed
		if ( m_nFirstFree == INVALID_REFCOUNT ) {
			m_vData[nIndex].nRefCount = INVALID_REFCOUNT;
			m_nFirstFree = nIndex;
		} else {
			m_vData[nIndex].nRefCount = -(m_nFirstFree+1);
			m_nFirstFree = nIndex;
		}
		m_nUsedCount--;
	}
		

	inline void clear( bool bFreeMem = false ) {
		if ( bFreeMem )
			m_vData.clear();
		else
			m_vData.resize(0);
		m_nFirstFree = INVALID_REFCOUNT;
		m_nUsedCount = 0;
	}


	inline unsigned int size() const { return m_nUsedCount; }
	inline unsigned int max_index() const { return (unsigned int)m_vData.size(); }

	inline Type & operator[]( int nIndex ) {
		lgASSERT( nIndex < (int)m_vData.size() && m_vData[nIndex].nRefCount > 0 && m_vData[nIndex].nRefCount != INVALID_REFCOUNT  );
		return m_vData[nIndex].vecData;
	}
	inline const Type & operator[]( int nIndex ) const {
		lgASSERT( nIndex < (int)m_vData.size() && m_vData[nIndex].nRefCount > 0 && m_vData[nIndex].nRefCount != INVALID_REFCOUNT  );
		return m_vData[nIndex].vecData;
	}

	std::string printData() const {
		std::ostrstream out;
		out << "[ F: ";
		if ( m_nFirstFree == INVALID_REFCOUNT )
			out << "X" << " VS: " <<  (int)m_vData.size() << " RS: " << m_nUsedCount << " ]" << std::endl;
		else
			out << m_nFirstFree << " VS: " <<  (int)m_vData.size() << " RS: " << m_nUsedCount << " ]" << std::endl;

		size_t nCount = m_vData.size();
		for ( unsigned int i = 0; i  < nCount; ++i ) {
			out << "  " << i << "[ " << *((int *)&m_vData[i].vecData) << "/" ;
			if  ( m_vData[i].nRefCount == INVALID_REFCOUNT )
				out << "X ]";
			else
				out << m_vData[i].nRefCount << " ]";
		}
		out << std::endl << '\0';
		return out.str();
	}

protected:
	struct RefEntry {
		int nRefCount;
		Type vecData;
	};
	std::vector< RefEntry > m_vData;

	unsigned int m_nUsedCount;
	int m_nFirstFree;

public:
	class item_iterator
	{
	public:
		inline item_iterator() { m_pCurrent = NULL; m_pLast = NULL;}

		inline item_iterator( item_iterator & copy ) {
			m_pCurrent = copy.m_pCurrent;
			m_pLast = copy.m_pLast;
		}
		
		inline item_iterator(const item_iterator & copy ) {
			m_pCurrent = copy.m_pCurrent;
			m_pLast = copy.m_pLast;
		}

		inline RefEntry & operator*() { 
			return *m_pCurrent;
		}

		inline item_iterator & operator++() {		// prefix
			goto_next();
			return *this;
		}
		inline item_iterator operator++(int) {		// postfix
			item_iterator copy(*this);
			goto_next();
			return copy;
		}

		inline bool operator==( item_iterator & r2 ) {
			return m_pCurrent == r2.m_pCurrent;
		}
		inline bool operator!=( item_iterator & r2 ) {
			return m_pCurrent != r2.m_pCurrent;
		}

	protected:
		inline void goto_next() {
			if ( m_pCurrent != m_pLast )
				m_pCurrent++;
			while ( (m_pCurrent->nRefCount <= 0 || m_pCurrent->nRefCount == INVALID_REFCOUNT) && m_pCurrent != m_pLast )
				m_pCurrent++;
		}

		inline item_iterator( RefEntry * pCurrent, RefEntry * pLast )
		{
			m_pCurrent = pCurrent;
			m_pLast = pLast;
			if ( (m_pCurrent->nRefCount <= 0 || m_pCurrent->nRefCount == INVALID_REFCOUNT) )
				goto_next();		// initialize
		}
		RefEntry * m_pCurrent;
		RefEntry * m_pLast;
		friend class RefCountedVector;
	};

	inline item_iterator begin_items() {
		return item_iterator( &m_vData.front(), &m_vData.back() + 1 );
	}
	inline item_iterator end_items() {
		return item_iterator( &m_vData.back() + 1, &m_vData.back() + 1 );
	}



	class index_iterator
	{
	public:
		inline index_iterator() { m_nIndex = 0; m_pCurrent = NULL; m_pLast = NULL;}

		inline index_iterator( const index_iterator & copy ) {
			m_nIndex = copy.m_nIndex;
			m_pCurrent = copy.m_pCurrent;
			m_pLast = copy.m_pLast;
		}

		inline int operator*() { 
			return m_nIndex;
		}

		inline index_iterator & operator++() {			// prefix
			goto_next();
			return *this;
		}
		inline index_iterator operator++(int) {		// postfix
			index_iterator copy(*this);
			goto_next();
			return copy;
		}

		inline bool operator==( index_iterator & r2 ) {
			return m_pCurrent == r2.m_pCurrent;
		}
		inline bool operator!=( index_iterator & r2 ) {
			return m_pCurrent != r2.m_pCurrent;
		}

	protected:
		inline void goto_next() {
			if ( m_pCurrent != m_pLast ) {
				m_pCurrent++;
				m_nIndex++;
			}
			while ( (m_pCurrent->nRefCount <= 0 || m_pCurrent->nRefCount == INVALID_REFCOUNT) && m_pCurrent != m_pLast ) {
				m_pCurrent++;
				m_nIndex++;
			}
		}

		inline index_iterator( int nIndex, const RefEntry * pCurrent, const RefEntry * pLast )
		{
			m_nIndex = nIndex;
			m_pCurrent = pCurrent;
			m_pLast = pLast;
			if ( m_pCurrent == m_pLast )
				;		// do nothing - finished!
			else if ( (m_pCurrent->nRefCount <= 0 || m_pCurrent->nRefCount == INVALID_REFCOUNT) )
				goto_next();		// initialize
		}
		int m_nIndex;
		const RefEntry * m_pCurrent;
		const RefEntry * m_pLast;
		friend class RefCountedVector;
	};

	inline index_iterator begin_indexes() const {
		if ( m_vData.empty() )
			return end_indexes();
		else
			return index_iterator( (int)0, &m_vData.front(), &m_vData.back() + 1 );
	}
	inline index_iterator end_indexes() const {
		if ( m_vData.size() == 0 ) 
			return index_iterator( 0, NULL, NULL );
		else
			return index_iterator( (int)m_vData.size(), &m_vData.back() + 1, &m_vData.back() + 1 );
//			return index_iterator( (int)m_vData.size(), &m_vData.back() + 1, &m_vData.back() + 1 );
	}
};




/*
 * memory-pool linked-list class
 */
 // [RMS: wouldn't it be easier to use an STL-like allcator pattern for this??]
template<class DataType>
class ListPool
{
public:
	struct Entry {
		DataType data;
		Entry * pNext;
		inline bool operator==( const Entry & e2 ) const { return data == e2.data; }
	};

	struct List {
		Entry * pFirst;
	};

	inline void Clear( bool bFreeMem ) {
		m_MemPool.ClearAll();
	}

	inline List GetList() const {
		List l;
		l.pFirst = NULL;
		return l;
	}
	
	inline void Insert( List & list, const DataType & insert ) {
		Entry * pNew = m_MemPool.Allocate();
		pNew->data = insert;
		pNew->pNext = NULL;

		if ( list.pFirst == NULL )
			list.pFirst = pNew;
		else {
			Entry * pCur = list.pFirst;
			while ( pCur->pNext != NULL )
				pCur = pCur->pNext;
			pCur->pNext = pNew;
		}
	}

	inline void Remove( List & list, const DataType & remove ) {
		Entry * pCur = list.pFirst;
		if ( ! pCur )
			return;
		if ( pCur->data == remove ) {
			list.pFirst = pCur->pNext;
			m_MemPool.Free(pCur);
		} else {
			Entry * pLast = pCur;
			pCur = pCur->pNext;
			while ( pCur != NULL ) {
				if ( pCur->data == remove ) {
					Entry * pTmp = pCur;	
					pLast->pNext = pCur->pNext;
					m_MemPool.Free( pTmp );
					return;
				}
				pLast = pCur;
				pCur = pCur->pNext;
			}
		}
	}

	inline void Clear( List & list ) {
		if ( ! list.pFirst )
			return;
		Entry * pCur = list.pFirst;
		while ( pCur->pNext != NULL ) {
			Entry * pTmp = pCur;
			pCur = pCur->pNext;
			m_MemPool.Free(pTmp);
		}
	}

	inline bool Find( List & list, const DataType & entry, Entry ** pEntry = NULL ) const {
		Entry * pCur = list.pFirst;
		while ( pCur != NULL ) {
			if ( pCur->data == entry ) {
				if ( pEntry )
					*pEntry = pCur;
				return true;
			}
			pCur = pCur->pNext;
		}
		return false;
	}

	inline bool Has( List & list, const DataType & entry ) const {
		Entry * pCur = list.pFirst;
		while ( pCur != NULL ) {
			if ( pCur->data == entry )
				return true;
			pCur = pCur->pNext;
		}
		return false;
	}

protected:
	MemoryPool<Entry> m_MemPool;
};


}  // end namespace rms

#pragma warning( pop )
