// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#ifndef __RMS_SPARSE_ARRAY_H__
#define __RMS_SPARSE_ARRAY_H__

// ignore annoying warning about dll-interface for vector that is not exposed...
#pragma warning( push )
#pragma warning( disable: 4251 )

#include "config.h"
#include <set>
#include "MemoryPool.h"
#include <cassert>




namespace rms {


// RMS TODO: replace std::set with a balanced binary tree (can easily
//  be balanced if we know bucket size). Do allocation via a mempool
//  shared with SparseArray...

template<class T>
struct SparseBucketEntry {
	unsigned int i;
	mutable T val;
	inline bool operator<( const SparseBucketEntry & e2 ) const
		{ return i < e2.i; }
	inline bool operator==( const SparseBucketEntry & e2 ) const
		{ return i == e2.i; }
};


template<class T, class EntryType = SparseBucketEntry<T> >
class SparseBucket
{

public:  
	typedef unsigned int Index;
	std::set< EntryType > m_vData;

public:

	inline void clear()
		{ m_vData.clear(); }

	inline bool empty() const
		{ return m_vData.empty(); }

	inline bool set( Index i, const T & v ) { 
		EntryType e; e.i = i; e.val = v; 
		std::pair<typename std::set<EntryType>::iterator, bool > pr = m_vData.insert(e);
		//if ( ! pr.second )
		//	(*pr.first).val = v;
		return pr.second;
	}

	inline bool remove( Index i ) {
		EntryType e; e.i = i; 
		std::pair<typename std::set<EntryType>::iterator, bool > pr = m_vData.erase(e); 
		return pr.second;
	}
		

	inline bool has( Index i ) const {
		EntryType e; e.i = i;
		
		return  m_vData.find(e) != m_vData.end();
		
		// std::set<EntryType>::const_iterator cur( m_vData.find(e) );
		// return (cur != m_vData.end());
	}

  // todo: what happens if there is no such element?
	inline T & get( Index i ) { 
		EntryType e; e.i = i;
		
		assert(1==0); // "not yet moved to Linux"
		
		// T foo = m_vData.find(e)->val;
		
		// return foo;
		
		// std::set<EntryType>::iterator cur( m_vData.find(e) );
		// return (*cur).val;
	}
	
	inline const T & get( Index i ) const { 
		EntryType e; e.i = i;
		return m_vData.find(e)->val;
		
		// std::set<EntryType>::const_iterator cur( m_vData.find(e) );
		// return (*cur).val;
	}

	inline T & operator[]( Index i ) {
		EntryType e; e.i = i;
		return m_vData.find(e).val;
//		std::set<EntryType>::iterator cur( m_vData.find(e) );
//		return (*cur).val;
	}
	inline const T & operator[]( Index i ) const {
		EntryType e; e.i = i;
		return m_vData.find(e).val;
		//std::set<EntryType>::iterator cur( m_vData.find(e) );
		//return (*cur).val;
	}

	typedef typename std::set<EntryType>::iterator iterator;

	inline iterator begin() { return m_vData.begin(); }
	inline iterator end() { return m_vData.end(); }
};


// [RMS: bucket size is currently hardcoded. Should probably be a variable...]
#define BUCKET_POW2 10
#define BUCKET_SIZE		(1 << BUCKET_POW2)
#define BUCKET_INDEX(k)  ( (k) >> BUCKET_POW2 )
#define BUCKET_MASK(k) ( (k) & 0x3FF )

template<class T, class SparseBucketType = SparseBucket<T> >
class SparseArray
{
public:
	typedef unsigned int Index;

	SparseArray( unsigned int nSize = 0 )
		{ resize(nSize); m_nCount = 0; }

	inline void clear( bool bFreeBuckets = true ) { 
		for ( unsigned int i = 0; i < m_vBuckets.size(); ++i ) 
			m_vBuckets[i].clear();
		m_nCount = 0;
		if ( bFreeBuckets )
			m_vBuckets.clear();
	}

	inline void resize( unsigned int nSize )
		{	int nBuckets = nSize / BUCKET_SIZE + ((nSize % BUCKET_SIZE == 0) ? 0 : 1);
			m_vBuckets.resize(nBuckets); }

	inline size_t size() const
		{ return m_nCount; }

	inline bool empty() const
		{ return m_nCount == 0; }

	inline void set( Index i, const T & v ) 
		{	unsigned int nBucket = BUCKET_INDEX(i);
	        lgASSERT( nBucket < m_vBuckets.size() );
			if ( m_vBuckets[ nBucket ].set( BUCKET_MASK(i), v ) ) ++m_nCount; }

	inline void set_and_grow( Index i, const T & v )
		{	unsigned int nBucket = BUCKET_INDEX(i);
			if ( nBucket >= m_vBuckets.size() )
				resize(i+1);
			if ( m_vBuckets[ nBucket ].set( BUCKET_MASK(i), v ) ) ++m_nCount; }


	inline void erase( Index i ) 
		{ lgASSERT( BUCKET_INDEX(i) < m_vBuckets.size() );
		  if ( m_vBuckets[ BUCKET_INDEX(i) ].erase( BUCKET_MASK(i) ) ) --m_nCount; }

	inline T & get( Index i )
		{	lgASSERT( BUCKET_INDEX(i) < m_vBuckets.size() );
			return m_vBuckets[ BUCKET_INDEX(i) ].get( BUCKET_MASK(i) ); }
	inline const T & get( Index i ) const
		{	lgASSERT( BUCKET_INDEX(i) < m_vBuckets.size() );
			return m_vBuckets[ BUCKET_INDEX(i) ].get( BUCKET_MASK(i) ); }

	inline bool has( Index i ) const
		{	lgASSERT( BUCKET_INDEX(i) < m_vBuckets.size() );
			return BUCKET_INDEX(i) < m_vBuckets.size() && m_vBuckets[ BUCKET_INDEX(i) ].has( BUCKET_MASK(i) ); }

	inline T & operator[]( Index i ) 
		{	lgASSERT( BUCKET_INDEX(i) < m_vBuckets.size() );
			return m_vBuckets[ BUCKET_INDEX(i) ].get( BUCKET_MASK(i) ); }
	inline const T & operator[]( Index i ) const
		{	lgASSERT( BUCKET_INDEX(i) < m_vBuckets.size() );
			return m_vBuckets[ BUCKET_INDEX(i) ].get( BUCKET_MASK(i) ); }


	/*
	 * iterators
	 */
	class iterator {
	public:
		inline iterator( const iterator & i2 ) {
			m_pArray = i2.m_pArray;
			m_nCurBucket = i2.m_nCurBucket;
			m_bcur = i2.m_bcur;
		}

		inline T & operator*() { return (*m_bcur).val; }

		inline iterator & operator++() {		// prefix
			goto_next();
			return *this;
		}
		inline iterator operator++(int) {		// postfix
			iterator copy(*this);
			goto_next();
			return copy;
		}

		inline bool operator==( const iterator & i2 ) {
			return ( m_pArray == i2.m_pArray &&  m_nCurBucket == i2.m_nCurBucket && m_bcur == i2.m_bcur );			// array & bucket must be the same for this to occur!
		}
		inline bool operator!=( const iterator & i2 ) {
			return ( m_pArray != i2.m_pArray ||  m_nCurBucket != i2.m_nCurBucket || m_bcur != i2.m_bcur );			// array & bucket must be the same for this to occur!
		}

		inline Index index() {
			return (m_nCurBucket * BUCKET_SIZE) + (*m_bcur).i;
		}


	protected:
		SparseArray<T> * m_pArray;
		Index m_nCurBucket;
		typename SparseBucketType::iterator m_bcur;
		friend class SparseArray<T>;

		inline iterator( SparseArray<T> * pArray, bool bStart ) {
			m_pArray = pArray;
			if ( bStart ) {
				m_nCurBucket = 0;
				m_bcur = m_pArray->m_vBuckets[0].begin();
				if ( m_bcur == m_pArray->m_vBuckets[0].end() )
					goto_next();
			} else {
				m_nCurBucket = (m_pArray->m_vBuckets.size() > 0) ? 
					(Index)(m_pArray->m_vBuckets.size()-1) : 0;
				m_bcur = m_pArray->m_vBuckets[m_nCurBucket].end();
			}
		}

		inline void goto_next() {
			if ( m_bcur == m_pArray->m_vBuckets[m_nCurBucket].end() ) {
				while ( (int)m_nCurBucket < (int)m_pArray->m_vBuckets.size()-1 && 
						m_bcur == m_pArray->m_vBuckets[m_nCurBucket].end() ) {
					m_nCurBucket++;
					m_bcur = m_pArray->m_vBuckets[m_nCurBucket].begin();
				} 
			} else {
				m_bcur++;

				// recursive hack to handle case where m_bcur just hit end(). is
				// there a cleaner way to write this ??
				if ( m_bcur == m_pArray->m_vBuckets[m_nCurBucket].end() )
					goto_next();
			}
		}
	};
	friend class SparseArray<T>::iterator;

	inline iterator begin() { return iterator(this, true); }
	inline iterator end() { return iterator(this, false); }

protected:
	DynamicVector< SparseBucketType > m_vBuckets;
	unsigned int m_nCount;
};



}


#endif // __RMS_SPARSE_ARRAY_H__
