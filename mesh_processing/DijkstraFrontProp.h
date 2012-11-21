// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#include "config.h"
#include "IGeometry.h"
#include <Wm4Vector3.h>
#include <limits>
namespace rms {

// TODO:
//   - support custom distance function
//   - support stopping criteria
//   - support point-sets
//   - fibonacci heap

template<class T>
class DijkstraFrontProp
{
public:
	DijkstraFrontProp() { m_pPointSource = NULL; m_pNbrSource = NULL; }
	~DijkstraFrontProp() {}

	void SetSource( IPositionSource<T> * pointSource, INeighbourSource<T> * nbrSource )
		{ m_pPointSource = pointSource;  m_pNbrSource = nbrSource; }

	//! if any filter verts are added, algorithm only propagates to/through filter verts
	void AppendFilterVerts( const std::set<T> & filter )
		{ m_filterSet.insert( filter.begin(), filter.end() ); }
	void AppendFilterVert( T vID )
		{ m_filterSet.insert( vID ); }

	//! if any terminate verts are added, algorithm halts when all terminate verts are reached
	void AppendTerminateVerts( const std::set<T> & terminate )
		{ m_terminateSet.insert( terminate.begin(), terminate.end() ); }
	void AppendTerminateVert( T vID )
		{ m_terminateSet.insert( vID ); }

	void Reset();

	void AppendStartValue( T vStart, float fStartValue = 0 );
	void AppendStartValues( const std::vector<T> & vStartValues, float fStartValue = 0);

	void FindAllDistances();
	

	struct VertInfo {
		T vID;
		float fMinDist;
		bool bFrozen;

		VertInfo() { fMinDist = 0; bFrozen = false; }
		inline bool operator<( const VertInfo & d2 ) const 
			{ return ( fMinDist < d2.fMinDist || (fMinDist == d2.fMinDist && vID < d2.vID) ); }
	};
	
	const std::vector<VertInfo> & GetResults() { return m_vInfo; }
	const std::vector<T> & GetOrder() { return m_vOrder; }
	
	float GetMaxDistance() { return m_vInfo[m_vOrder.back()].fMinDist; }

protected:
	IPositionSource<T> * m_pPointSource;
	INeighbourSource<T> * m_pNbrSource;

	std::set<T> m_filterSet;
	std::set<T> m_terminateSet;

	struct StartValue {
		T vID;
		float fValue;
		StartValue() { fValue = 0.0f; }
		StartValue( T id, float val ) { vID = id; fValue = val; }
	};
	std::vector<StartValue> m_vStartSet;

	std::vector<VertInfo> m_vInfo;
	std::vector<T> m_vOrder;

	

	// data structures for running Dijkstra's algorithm


	struct VertWrapper {
		VertWrapper( VertInfo * p ) { pVert = p; }
		VertInfo * pVert;
		inline bool operator< ( const VertWrapper & v2 ) const {
			if ( pVert->fMinDist < v2.pVert->fMinDist || (pVert->fMinDist == v2.pVert->fMinDist && pVert->vID < v2.pVert->vID) )
				return true;
			return false;
		}
	};


	typedef std::set<VertWrapper> DJQueue;
	typedef std::vector<VertInfo> DJVector;

	DJQueue m_vQueue;
	void UpdateQueue(T vID);
};




template <class T>
void DijkstraFrontProp<T>::Reset()
{
	m_vStartSet.resize(0);
	m_vOrder.resize(0);
	m_filterSet.clear();
	m_terminateSet.clear();
}


template <class T>
void DijkstraFrontProp<T>::AppendStartValue( T vStart, float fStartValue )
{
	if ( ! m_filterSet.empty() && m_filterSet.find(vStart) == m_filterSet.end() )
		lgBreakToDebugger();		// vertex outside filter set was touched...
	m_vStartSet.push_back( StartValue(vStart, fStartValue) );
}

template <class T>
void DijkstraFrontProp<T>::AppendStartValues( const std::vector<T> & vStartValues, float fStartValue )
{
	size_t nCount = vStartValues.size();
	for ( unsigned int i = 0; i < nCount; ++i )
		AppendStartValue( vStartValues[i], fStartValue );
}


template <class T>
void DijkstraFrontProp<T>::UpdateQueue(T vID)
{
	std::vector<T> vNbrs;
	m_pNbrSource->GetNeighbours(vID, vNbrs);

	Wml::Vector3f vVtx, vNbrVtx;
	m_pPointSource->GetPosition(vID, vVtx);
	VertInfo & vVert = m_vInfo[vID];

	size_t nNbrs = vNbrs.size();
	for ( unsigned int k = 0; k < nNbrs; ++k ) {
		T vNbrID = vNbrs[k];
		if ( ! m_filterSet.empty() && m_filterSet.find(vNbrID) == m_filterSet.end() )
			continue;

		VertInfo & vNbr = m_vInfo[vNbrID];
		if ( vNbr.bFrozen )
			continue;			// already done


		m_pPointSource->GetPosition(vNbrID, vNbrVtx);
		float fDist = vVert.fMinDist + ( (vNbrVtx-vVtx).Length() );

		if ( fDist < vNbr.fMinDist ) {
			VertWrapper wrapv(&vNbr);

			typename DJQueue::iterator found( m_vQueue.find( wrapv ) );
			if ( found != m_vQueue.end() )
				m_vQueue.erase(found);
			vNbr.fMinDist = fDist;
			m_vQueue.insert( wrapv );
		}
	}
}



template <class T>
void DijkstraFrontProp<T>::FindAllDistances()
{
	lgASSERT(m_pNbrSource->MaxID() == m_pPointSource->MaxID());
	T nMaxID = m_pNbrSource->MaxID();
	m_vInfo.resize(0);
	m_vInfo.resize( nMaxID );

	std::set<T> terminate = m_terminateSet;
	bool bEarlyTerminate = ! terminate.empty();

	// initialize vertices (only process verts in filter set if we know them)
	if ( m_filterSet.empty() ) {
		for ( unsigned int k = 0; k < nMaxID; ++k ) {
			T vID = (T)k;
			m_vInfo[vID].vID = vID;
			m_vInfo[vID].fMinDist = std::numeric_limits<float>::max()/2;
			m_vInfo[vID].bFrozen = false;
		}
	} else {
		typename std::set<T>::iterator curv(m_filterSet.begin()), endv(m_filterSet.end());
		while ( curv != endv ) {
			T vID = *curv++;
			m_vInfo[vID].vID = vID;
			m_vInfo[vID].fMinDist = std::numeric_limits<float>::max()/2;
			m_vInfo[vID].bFrozen = false;
		}
	}

	// insert all start verts and do queue updates
	m_vQueue.clear();
	size_t nStart = m_vStartSet.size();
	for ( unsigned int i = 0; i < nStart; ++i ) {
		T vID = m_vStartSet[i].vID;
		VertInfo * pVert = & m_vInfo[vID];
		pVert->fMinDist = m_vStartSet[i].fValue;
		pVert->bFrozen = true;

		m_vQueue.insert( VertWrapper( pVert ) );

		UpdateQueue( vID );
	}

	// ok now pop verts one-by-one
	bool bHalt = false;
	while (! m_vQueue.empty() && ! bHalt ) {
		
		VertWrapper remove = * m_vQueue.begin();
		remove.pVert->bFrozen = true;
		m_vOrder.push_back( remove.pVert->vID );

		if ( bEarlyTerminate ) {
			terminate.erase( remove.pVert->vID );
			if ( terminate.empty() ) {
				bHalt = true;
				continue;
			}
		}

		if ( ! m_filterSet.empty() && m_filterSet.find(remove.pVert->vID) == m_filterSet.end() )
			lgBreakToDebugger();		// vertex outside filter set was touched...
	
		m_vQueue.erase( m_vQueue.begin() );

		UpdateQueue( remove.pVert->vID );
	}
}




}   // end namespace rms
