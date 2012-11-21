// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#ifndef __RMS_BITSET_H__
#define __RMS_BITSET_H__

// ignore annoying warning about dll-interface for vector that is not exposed...
#pragma warning( push )
#pragma warning( disable: 4251 )

#include "config.h"

namespace rms {


class BitSet
{
public:
	BitSet( unsigned int nSize = 0 )
		{ resize(nSize); m_nCount = 0; }

	inline void clear(  ) 
	    { resize(0); m_nCount = 0; }

	inline void resize( size_t nSize )
		{ m_vBits.resize(0); m_vBits.resize(nSize,false); }

	inline size_t size() const
		{ return m_vBits.size(); }

	inline bool empty() const
		{ return m_vBits.empty(); }

	inline void set( unsigned int i, bool bValue ) 
		{	if ( m_vBits[i] && bValue == false )	--m_nCount;
	        if ( !m_vBits[i] && bValue == true ) ++m_nCount;
	        m_vBits[i] = bValue;
		}

	inline unsigned int set_count() const
		{ return m_nCount; }

	inline bool get( unsigned int i ) const
		{ return m_vBits[i]; }

	inline bool operator[]( unsigned int i ) const
		{ return m_vBits[i]; }

	inline void flip() {
		size_t count = m_vBits.size();
		for ( unsigned int k = 0; k < count; ++k ) m_vBits[k] = ! m_vBits[k];
	}

	inline void or( BitSet & set2 ) {
		size_t nMin = std::min(size(), set2.size());
		for ( unsigned int k = 0; k < nMin; ++k )
			m_vBits[k] = m_vBits[k] || set2.m_vBits[k];
	}
	inline void and( BitSet & set2 ) {
		size_t nMin = std::min(size(), set2.size());
		for ( unsigned int k = 0; k < nMin; ++k )
			m_vBits[k] = m_vBits[k] && set2.m_vBits[k];
	}
	inline void xor( BitSet & set2 ) {
		size_t nMin = std::min(size(), set2.size());
		for ( unsigned int k = 0; k < nMin; ++k )
			m_vBits[k] = m_vBits[k] != set2.m_vBits[k];
	}

	void convert( std::set<unsigned int> & v, bool bInvert = false) {
		size_t nCount = m_vBits.size();
		for ( unsigned int i = 0; i < nCount; ++i ) {
			if ( (m_vBits[i] && !bInvert) || (!m_vBits[i] && bInvert) )
				v.insert(i);
		}
	}
	void convert( std::vector<unsigned int> & v, bool bInvert = false) {
		size_t nCount = m_vBits.size();
		for ( unsigned int i = 0; i < nCount; ++i ) {
			if ( (m_vBits[i] && !bInvert) || (!m_vBits[i] && bInvert) )
				v.push_back(i);
		}
	}

protected:
	std::vector<bool> m_vBits;
	unsigned int m_nCount;
};




}


#endif // __RMS_BITSET_H__