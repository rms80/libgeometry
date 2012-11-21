// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#include "config.h"
#include "IGeometry.h"
#include <vector>


namespace rms {


template <class T>
class NeighbourCache : public INeighbourSource<T>
{
public:
	NeighbourCache(void) { m_pSource = NULL; }
	~NeighbourCache(void) { m_pSource = NULL; }

	void SetSource( INeighbourSource<T> * pSource ) { 
		m_pSource = pSource; 
		m_nSize = m_pSource->MaxID();
		m_vNeighbours.resize(0);
		m_vNeighbours.resize(m_nSize);
		m_vInitialized.resize(0);
		m_vInitialized.resize(m_nSize, false);
		nComputed = 0;
	}

	struct NeighbourSet {
		std::vector<T> vNbrs;
	};

	const NeighbourSet & GetNeighbours( T nID ) {
		NeighbourSet & s = m_vNeighbours[nID];
		if ( ! m_vInitialized[nID] ) {
			m_pSource->GetNeighbours(nID, s.vNbrs);
			m_vInitialized[nID] = true;
			++nComputed;
		}
		return s;
	}


	/* 
	 * INeighbourSource interface
	 */
	virtual void GetNeighbours( T nID, std::vector<T> & vNbrIDs ) 
		{ vNbrIDs = GetNeighbours(nID).vNbrs; }
	virtual T MaxID() { return m_pSource->MaxID(); }
	virtual bool IsOrdered() { return m_pSource->IsOrdered(); }


protected:
	INeighbourSource<T> * m_pSource;
	T m_nSize;
	std::vector<NeighbourSet> m_vNeighbours;
	std::vector<bool> m_vInitialized;

	unsigned int nComputed;
};



}   // end namespace rms