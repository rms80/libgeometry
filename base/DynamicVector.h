// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#ifndef _RMS_DYNAMIC_VECTOR_H
#define _RMS_DYNAMIC_VECTOR_H

#include <cstddef> // 'size_t'
#include "config.h"
#include <vector>

namespace rms
{


template<class Type>
class DataSegment {
public:
	Type * pData;
	size_t nSize;
	size_t nCur;
	DataSegment() { pData = NULL; }
	~DataSegment() { }
};


template<class Type>
class DynamicVector
{
public:
	DynamicVector(unsigned int nSegmentSize = 0);
	DynamicVector(const DynamicVector & copy);
	virtual ~DynamicVector();

	const DynamicVector & operator=( const DynamicVector & copy );

	void clear( bool bFreeSegments = false );
	void resize( size_t nCount );
	void resize( size_t nCount, const Type & init_value );

	size_t size() const;

	void push_back( const Type & data );
	Type * push_back();
	void push_back( const DynamicVector<Type> & data );

	Type & operator[]( unsigned int nIndex );
	const Type & operator[]( unsigned int nIndex ) const;

protected:
	unsigned int m_nSegmentSize;

	unsigned int m_nCurSeg;

	std::vector< DataSegment<Type> > m_vSegments;

	Type * allocate_element();
};




template <class Type>
DynamicVector<Type>::DynamicVector(unsigned int nSegmentSize)
{
	if ( nSegmentSize == 0 )
		m_nSegmentSize = (1 << 16) / sizeof(Type);		// 64k
	else
		m_nSegmentSize = nSegmentSize;

	m_vSegments.resize(1);
	m_vSegments[0].pData = new Type[ m_nSegmentSize ];
	m_vSegments[0].nSize = m_nSegmentSize;
	m_vSegments[0].nCur = 0;

	m_nCurSeg = 0;
}

template <class Type>
DynamicVector<Type>::DynamicVector(const DynamicVector<Type> & copy)
{
	*this = copy;
}


template <class Type>
DynamicVector<Type>::~DynamicVector()
{
	size_t nCount = m_vSegments.size();
	for (unsigned int i = 0; i < nCount; ++i)
		delete [] m_vSegments[i].pData;
}

template <class Type>
const DynamicVector<Type> & DynamicVector<Type>::operator=( const DynamicVector & copy )
{
	// if segments are the same size, we don't need to re-allocate any existing ones  (woot!)
	if ( m_nSegmentSize != copy.m_nSegmentSize )
		clear(true);

	m_nSegmentSize = copy.m_nSegmentSize;
	m_nCurSeg = copy.m_nCurSeg;

	// allocate memory (or discard exisiting memory) for segments
	resize( copy.size() );

	// copy segment contents
	size_t nSegs = copy.m_vSegments.size();
	for ( unsigned int k = 0; k < nSegs; ++k ) {
    #ifdef WIN32	
		  memcpy_s( m_vSegments[k].pData, m_nSegmentSize*sizeof(Type), copy.m_vSegments[k].pData, m_nSegmentSize*sizeof(Type) );
		#else
		  memcpy( m_vSegments[k].pData, copy.m_vSegments[k].pData, m_nSegmentSize*sizeof(Type) );
		#endif
		m_vSegments[k].nSize = m_nSegmentSize;
		m_vSegments[k].nCur = copy.m_vSegments[k].nCur;
	}
	return *this;
}


template <class Type>
void DynamicVector<Type>::clear( bool bFreeSegments )
{
	size_t nCount = m_vSegments.size();
	for (unsigned int i = 0; i < nCount; ++i) 
		m_vSegments[i].nCur = 0;

	if (bFreeSegments) {
		for (unsigned int i = 1; i < nCount; ++i)
			delete [] m_vSegments[i].pData;
		m_vSegments.resize(1);
	}

	m_nCurSeg = 0;
}


template <class Type>
void DynamicVector<Type>::resize( size_t nCount )
{
	// figure out how many segments we need
	unsigned int nNumSegs = 1 + (unsigned int)nCount / m_nSegmentSize;

	// figure out how many are currently allocated...
	size_t nCurCount = m_vSegments.size();

	// erase extra segments memory
	for ( unsigned int i = nNumSegs; i < nCurCount; ++i )
		delete [] m_vSegments[i].pData;

	// resize to right number of segments
	m_vSegments.resize(nNumSegs);

	// allocate new segments
	for (unsigned int i = (unsigned int)nCurCount; i < nNumSegs; ++i) {
		m_vSegments[i].pData = new Type[ m_nSegmentSize ];
		m_vSegments[i].nSize = m_nSegmentSize;
		m_vSegments[i].nCur = 0;
	}

	// mark full segments as used
	for (unsigned int i = 0; i < nNumSegs-1; ++i)
//		m_vSegments[i-1].nCur = m_nSegmentSize;
		m_vSegments[i].nCur = m_nSegmentSize;

	// mark last segment
	m_vSegments[nNumSegs-1].nCur = nCount - (nNumSegs-1)*m_nSegmentSize;

	m_nCurSeg = nNumSegs-1;
}

template <class Type>
void DynamicVector<Type>::resize( size_t nCount, const Type & init_value )
{
	size_t nCurSize = size();
	resize(nCount);
	for ( size_t nIndex = nCurSize; nIndex < nCount; ++nIndex ) 
		m_vSegments[ nIndex / m_nSegmentSize ].pData[ nIndex % m_nSegmentSize ] = init_value;
}


template <class Type>
size_t  DynamicVector<Type>::size() const
{
	return (m_nCurSeg)*m_nSegmentSize + m_vSegments[m_nCurSeg].nCur;
}


template <class Type>
Type * DynamicVector<Type>::allocate_element()
{
	DataSegment<Type> & seg = m_vSegments[m_nCurSeg];
	if ( seg.nCur == seg.nSize ) {
		if ( m_nCurSeg == m_vSegments.size() - 1 ) {
			m_vSegments.resize( m_vSegments.size() + 1 );
			DataSegment<Type> & newSeg = m_vSegments.back();
			newSeg.pData = new Type[ m_nSegmentSize ];
			newSeg.nSize = m_nSegmentSize;
			newSeg.nCur = 0;
		}
		m_nCurSeg++;
	}
	DataSegment<Type> & returnSeg = m_vSegments[m_nCurSeg];
	return & returnSeg.pData[ returnSeg.nCur++ ];
}

template <class Type>
void DynamicVector<Type>::push_back( const Type & data )
{
	Type * pNewElem = allocate_element();
	*pNewElem = data;
}

template <class Type>
Type * DynamicVector<Type>::push_back()
{
	return allocate_element();
}

template <class Type>
void DynamicVector<Type>::push_back( const DynamicVector<Type> & data )
{
	// [RMS TODO] it would be a lot more efficient to use memcopies here...
	size_t nSize = data.size();
	for ( unsigned int k = 0; k < nSize; ++k )
		push_back( data[k] );
}



template <class Type>
Type & DynamicVector<Type>::operator[]( unsigned int nIndex )
{
	return m_vSegments[ nIndex / m_nSegmentSize ].pData[ nIndex % m_nSegmentSize ];
}

template <class Type>
const Type & DynamicVector<Type>::operator[]( unsigned int nIndex ) const
{
	return m_vSegments[ nIndex / m_nSegmentSize ].pData[ nIndex % m_nSegmentSize ];
}


} // end namespace rms


#endif _RMS_DYNAMIC_VECTOR_H
