// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#include "DistanceCache.h"
#include <limits>

using namespace rms;

float DistanceCache::INVALID_VALUE = std::numeric_limits<float>::min();

DistanceCache::DistanceCache(bool bUseSparseArray)
{
	m_nSize = 0;
	m_pCalculator = NULL;
	m_bUseSparseArray = bUseSparseArray;
}

DistanceCache::~DistanceCache(void)
{
}


void DistanceCache::Resize(unsigned int nSize)
{
	if ( m_bUseSparseArray ) {
		m_vSparseDistances.clear(true);
		m_vSparseDistances.resize(nSize*nSize);
	} else {
		m_vDistances.resize(0);
		m_vDistances.resize(nSize*nSize, INVALID_VALUE);
	}

	m_nSize = nSize;
}

//! clears current values
void DistanceCache::SetUseSparse(bool bEnable)
{
	if ( m_bUseSparseArray && !bEnable ) {
		m_vSparseDistances.clear(true);
		m_vDistances.resize(0);
		m_vDistances.resize(m_nSize*m_nSize, INVALID_VALUE);
		m_bUseSparseArray = false;
	} else if ( !m_bUseSparseArray && bEnable ) {
		m_vDistances.clear(true);
		m_vSparseDistances.resize(m_nSize*m_nSize);
		m_bUseSparseArray = true;
	}
}


void DistanceCache::SetCalculator( DistanceCalculator * pCalc )
{
	m_pCalculator = pCalc;
}


float DistanceCache::GetValue( unsigned int i1, unsigned int i2 )
{
	if ( i1 > i2 ) { unsigned int tmp = i2; i2 = i1; i1 = tmp; }
	float fV = 0;
	if ( m_bUseSparseArray ) {
		if ( ! m_vSparseDistances.has(i1*m_nSize + i2) && m_pCalculator) {
			fV = m_pCalculator->GetDistance(i1,i2);
			SetValue(i1,i2,fV);
		} else
			fV = m_vSparseDistances.get(i1*m_nSize + i2);
	} else {
		fV = m_vDistances[i1*m_nSize + i2];
		if ( fV == INVALID_VALUE && m_pCalculator ) {
			fV = m_pCalculator->GetDistance(i1,i2);
			SetValue(i1,i2,fV);
		}
	}
	return fV;	
}

void DistanceCache::SetValue( unsigned int i1, unsigned int i2, float fValue )
{
	if ( i1 > i2 ) { unsigned int tmp = i2; i2 = i1; i1 = tmp; }
	if ( m_bUseSparseArray ) {
		m_vSparseDistances.set(i1*m_nSize + i2, fValue);
	} else {
		m_vDistances[i1*m_nSize + i2] = fValue;
	}
}
