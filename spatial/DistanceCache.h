// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#include "config.h"
#include <vector>
#include <SparseArray.h>


// [TODO] template DistanceCalculator on ID type  (and then template this class)


namespace rms {

class DistanceCache
{
public:
	static float INVALID_VALUE;

	DistanceCache(bool bUseSparseArray = false);
	~DistanceCache(void);

	void Resize(unsigned int nSize);

	float GetValue( unsigned int i1, unsigned int i2 );
	void SetValue( unsigned int i1, unsigned int i2, float fValue );

	//! clears current values
	void SetUseSparse(bool bEnable);

	class DistanceCalculator {
		public:
		virtual float GetDistance( unsigned int i1, unsigned int i2 ) = 0;
	};
	void SetCalculator( DistanceCalculator * pCalc );

protected:
	unsigned int m_nSize;
	
	bool m_bUseSparseArray;
	DynamicVector<float> m_vDistances;
	//std::vector<float> m_vDistances;
	SparseArray<float> m_vSparseDistances;

	DistanceCalculator * m_pCalculator;
};



}   // end namespace rms