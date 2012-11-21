// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#ifndef __LIBGEOMETRY_VERTEX_MAP_H__
#define __LIBGEOMETRY_VERTEX_MAP_H__

#include "config.h"
#include "IMesh.h"
#include "SparseArray.h"

namespace rms {

// [RMS] warning: this is not completely templated, because we assume that IMesh::InvalidID is
//   a valid value of type T. So T is actually just always an unsigned int, and this template is pointless....

template <class T>
class IDMap
{
public:
	enum MapType {
		DenseMap,
		SparseMap,
		ShiftMap
	};

	IDMap() { m_eType = DenseMap; }
	IDMap( bool bSparse ) { m_eType = (bSparse) ? SparseMap : DenseMap; }
	IDMap( unsigned int nSize, bool bSparse = false ) { 
		m_eType = (bSparse) ? SparseMap : DenseMap;
		Resize(nSize, nSize); 
	}
	IDMap( unsigned int nOldSize, unsigned int nNewSize, bool bSparse = false ) { 
		m_eType = (bSparse) ? SparseMap : DenseMap;
		Resize(nOldSize, nNewSize); 
	}
	IDMap( MapType eType ) {
		m_eType = eType;
		if ( eType == ShiftMap )
			m_nShift = 0;
	}

	void Clear() {
		if ( m_eType == ShiftMap )
			lgBreakToDebugger();
		else if ( m_eType == SparseMap ) {
			vToNew_Sparse.clear(false);
			vToOld_Sparse.clear(false);
		} else {
			vToNew.resize(0);
			vToOld.resize(0);
		}
	}

	inline void Resize( unsigned int nOldSize, unsigned int nNewSize ) {
		if ( m_eType == ShiftMap )
			lgBreakToDebugger();
		else if ( m_eType == SparseMap) {
			vToNew_Sparse.clear(false);
			vToNew_Sparse.resize(nOldSize);
			vToOld_Sparse.clear(false);
			vToOld_Sparse.resize(nNewSize);
		} else {
			vToNew.resize(0);
			vToNew.resize(nOldSize, IMesh::InvalidID);
			vToOld.resize(0);
			vToOld.resize(nNewSize, IMesh::InvalidID);
		}
	}
	inline size_t OldSize() const {
		if ( m_eType == ShiftMap ) {
			lgBreakToDebugger();
			return 0;
		} else
			return (m_eType == SparseMap) ? vToNew_Sparse.size() : vToNew.size(); 
	}
	inline size_t NewSize() const {
		if ( m_eType == ShiftMap ) {
			lgBreakToDebugger();
			return 0;
		} else
			return (m_eType == SparseMap) ? vToOld_Sparse.size() : vToOld.size(); 
	}

	void SetShift( int nShift ) {
		if ( m_eType != ShiftMap )
			lgBreakToDebugger();
		m_nShift = nShift;
	}

	inline void SetMap( T vOld, T vNew ) {
		if ( m_eType == ShiftMap ) 
			lgBreakToDebugger();
		if (m_eType == SparseMap) {
			vToNew_Sparse.set(vOld, vNew);
			if ( vNew != IMesh::InvalidID )
				vToOld_Sparse.set(vNew, vOld);
		} else {
			vToNew[vOld] = vNew;
			if ( vNew != IMesh::InvalidID )
				vToOld[vNew] = vOld;
		}
	}

	inline T GetNew( T vOld ) const {
		if ( m_eType == ShiftMap )
			return vOld + m_nShift;
		else if (m_eType == SparseMap)
			return ( vToNew_Sparse.has(vOld) ) ? vToNew_Sparse.get(vOld) : IMesh::InvalidID;
		else
			return vToNew[vOld];
	}
	inline T GetOld( T vNew ) const {
		if ( m_eType == ShiftMap )
			return vNew - m_nShift;
		else if (m_eType == SparseMap)
			return ( vToOld_Sparse.has(vNew) ) ? vToOld_Sparse.get(vNew) : IMesh::InvalidID;
		else
			return vToOld[vNew];
	}

private:
	MapType m_eType;
	std::vector<T> vToNew;
	std::vector<T> vToOld;
	SparseArray<T> vToNew_Sparse;
	SparseArray<T> vToOld_Sparse;
	int m_nShift;
};


typedef IDMap<IMesh::VertexID> VertexMap;
typedef IDMap<IMesh::VertexID> TriangleMap;
typedef IDMap<unsigned int> UVMap;


} // namespace rms


#endif  // __LIBGEOMETRY_VERTEX_MAP_H__