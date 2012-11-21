// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#ifndef __LIBGEOMETRY_VF_MESH_MERGE_H__
#define __LIBGEOMETRY_VF_MESH_MERGE_H__

#include "config.h"
#include "VFTriangleMesh.h"
#include "VFMeshMask.h"


namespace rms {

class VFMeshMerge : public IMesh
{
public:
	VFMeshMerge( );

	struct VertexPair {
		VertexID baseVID;
		VertexID mergeVID;
		VertexPair() {baseVID = InvalidID; mergeVID = InvalidID; }
		VertexPair( VertexID base, VertexID merge ) { baseVID = base; mergeVID = merge; }
	};
	void Initialize( IMesh * pBaseMesh, VFMeshMask * pMask, IMesh * pMergeMesh,
					 VertexPair * vPairs, unsigned int nPairs );

	IMesh * GetBaseMesh() { return m_pBaseMesh; }
	VFMeshMask * GetBaseMask() { return m_pBaseMask; }
	IMesh * GetMergeMesh() { return m_pMergeMesh; }

	void Clear( bool bFreeMem = false ) 
		{ m_vBaseToMerge.clear(bFreeMem);  m_vMergeToBase.clear(bFreeMem); 
		  m_nMaxVID = 0;  m_nMaxTID = 0; }

	//! determine whether or not a triangle ID is part of the base mesh...
	inline bool IsMergeTID( TriangleID tID ) const { 
		return tID >= m_nMaxBaseTID; }

/*
 * IMesh read interface (mandatory)
 */
	virtual void GetVertex( VertexID vID, Wml::Vector3f & vVertex, Wml::Vector3f * pNormal = NULL ) const;
	virtual void GetNormal( VertexID vID, Wml::Vector3f & vNormal ) const;
	virtual unsigned int GetVertexCount() const
		{ return m_nMaxVID; }
	virtual unsigned int GetMaxVertexID() const
		{ return m_nMaxVID; }
	virtual bool IsVertex( VertexID vID ) const
		{ lgBreakToDebugger(); return false; }

	virtual void GetTriangle( TriangleID tID, VertexID vTriangle[3]  ) const;
	virtual void GetTriangle( TriangleID tID, Wml::Vector3f vTriangle[3], Wml::Vector3f * pNormals = NULL ) const;
	virtual unsigned int GetTriangleCount() const
		{ return m_nMaxTID; }
	virtual unsigned int GetMaxTriangleID() const
		{ return m_nMaxTID; }
	virtual bool IsTriangle( TriangleID tID ) const
		{ lgBreakToDebugger(); return false; }

/*
 * IMesh mesh info interface - has default implementation
 */
	//! initialize vertex neighbour iteration
	virtual void BeginVtxTriangles( VtxNbrItr & v ) const;

	//! (possibly) un-ordered iteration around one-ring of a vertex. Returns InvalidID when done
	virtual TriangleID GetNextVtxTriangle( VtxNbrItr & v ) const;

	//! determine if a vertex is on the mesh boundary (if there is one)  (depends on GetNextVtxTriangle)
	virtual bool IsBoundaryVertex( VertexID vID ) const;

	//! iterate around neighbour triangles of vID, sending each to callback
	virtual void NeighbourIteration( VertexID vID, NeighborTriCallback * pCallback );


/*
 * basic iterators
 */
	class vertex_iterator {
	public:
		inline vertex_iterator( const vertex_iterator & i2 ) 
			: m_pMesh(i2.m_pMesh),  m_bInBase(i2.m_bInBase), 
			m_base_cur(i2.m_base_cur),  	m_base_end(i2.m_base_end),
			m_merge_cur(i2.m_merge_cur),	m_merge_end(i2.m_merge_end)
			{ }

		inline const vertex_iterator & operator=( const vertex_iterator & i2 ) { 	
				m_pMesh = i2.m_pMesh;
				m_base_cur = i2.m_base_cur;		m_base_end = i2.m_base_end;
				m_merge_cur = i2.m_merge_cur;	m_base_cur = i2.m_base_cur;
				m_bInBase = i2.m_bInBase;		return *this;
			}
		
		inline VertexID operator*() { 
			return (m_bInBase) ? *m_base_cur : m_pMesh->FromMergeVID(*m_merge_cur); }
		

		inline void operator++(int) { // postfix  (no return to avoid copies)
			goto_next(); }
		inline vertex_iterator & operator++() { // prefix
			goto_next();  return *this; }

		inline bool operator==( const vertex_iterator & i2 ) { 
			if ( m_bInBase && i2.m_bInBase )
				return m_base_cur == i2.m_base_cur;
			else if ( ! ( m_bInBase && i2.m_bInBase ) )
				return m_merge_cur == i2.m_merge_cur;
			else
				return false;
		}
		inline bool operator!=( const vertex_iterator & i2 ) { 
			return ! ( *this == i2 ); }

	protected:
		VFMeshMerge * m_pMesh;
		IMesh::IVtxIterator m_base_cur;
		IMesh::IVtxIterator m_base_end;
		IMesh::IVtxIterator m_merge_cur;
		IMesh::IVtxIterator m_merge_end;
		bool m_bInBase;

		inline void goto_next() {
			if ( m_bInBase ) {
				m_base_cur++;
				if ( m_base_cur == m_base_end )
					m_bInBase = false;
			} else
				m_merge_cur++;
		}
	
		//! if bStart, return start iterator, else return end iterator
		inline vertex_iterator( VFMeshMerge * pMesh, bool bStart ) 
			: m_pMesh(pMesh), 
			m_base_cur(pMesh->m_pBaseMesh->BeginIVertices()), 
			m_base_end(pMesh->m_pBaseMesh->EndIVertices()),
			m_merge_cur(pMesh->m_pMergeMesh->BeginIVertices()), 
			m_merge_end(pMesh->m_pMergeMesh->EndIVertices())
			{
				m_bInBase = (bStart) ? true : false;
				if ( ! bStart ) m_merge_cur = m_merge_end;
			}
		friend class VFMeshMerge;		
	};
	friend class vertex_iterator;



	class triangle_iterator {
	public:
		inline triangle_iterator( const triangle_iterator & i2 ) 
			: m_pMesh(i2.m_pMesh),  m_bInBase(i2.m_bInBase), 
			m_base_cur(i2.m_base_cur),  	m_base_end(i2.m_base_end),
			m_merge_cur(i2.m_merge_cur),	m_merge_end(i2.m_merge_end)
			{ }

		inline const triangle_iterator & operator=( const triangle_iterator & i2 ) { 	
				m_pMesh = i2.m_pMesh;
				m_base_cur = i2.m_base_cur;		m_base_end = i2.m_base_end;
				m_merge_cur = i2.m_merge_cur;	m_base_cur = i2.m_base_cur;
				m_bInBase = i2.m_bInBase;		return *this;
			}
		
		inline TriangleID operator*() { 
			return (m_bInBase) ? *m_base_cur : m_pMesh->FromMergeTID(*m_merge_cur); }

		inline void operator++(int) { // postfix  (no return to avoid copies)
			goto_next(); }
		inline triangle_iterator & operator++() { // prefix
			goto_next();  return *this; }

		inline bool operator==( const triangle_iterator & i2 ) { 
			if ( m_bInBase && i2.m_bInBase )
				return m_base_cur == i2.m_base_cur;
			else if ( ! ( m_bInBase && i2.m_bInBase ) )
				return m_merge_cur == i2.m_merge_cur;
			else
				return false;
		}
		inline bool operator!=( const triangle_iterator & i2 ) { 
			return ! ( *this == i2 ); }

	protected:
		VFMeshMerge * m_pMesh;
		IMesh::ITriIterator m_base_cur;
		IMesh::ITriIterator m_base_end;
		IMesh::ITriIterator m_merge_cur;
		IMesh::ITriIterator m_merge_end;
		bool m_bInBase;

		inline void goto_next() {
			if ( m_bInBase ) {
				m_base_cur++;
				if ( m_base_cur == m_base_end )
					m_bInBase = false;
			} else
				m_merge_cur++;
		}
	
		//! if bStart, return start iterator, else return end iterator
		inline triangle_iterator( VFMeshMerge * pMesh, bool bStart ) 
			: m_pMesh(pMesh), 
			//m_base_cur(pMesh->m_pBaseMesh->BeginITriangles()), 
			//m_base_end(pMesh->m_pBaseMesh->EndITriangles()),
			m_base_cur(pMesh->m_pBaseMask->BeginITriangles()), 
			m_base_end(pMesh->m_pBaseMask->EndITriangles()),
			m_merge_cur(pMesh->m_pMergeMesh->BeginITriangles()), 
			m_merge_end(pMesh->m_pMergeMesh->EndITriangles())
			{
				m_bInBase = (bStart) ? true : false;
				if ( ! bStart ) m_merge_cur = m_merge_end;
			}
		friend class VFMeshMerge;		
	};
	friend class triangle_iterator;


	inline vertex_iterator begin_vertices()   { return vertex_iterator(this, true); }
	inline vertex_iterator end_vertices()     { return vertex_iterator(this, false); }

	inline triangle_iterator begin_triangles()  { return triangle_iterator(this, true); }
	inline triangle_iterator end_triangles()    { return triangle_iterator(this, false); }


protected:
	IMesh * m_pBaseMesh;
	VFMeshMask * m_pBaseMask;

	IMesh * m_pMergeMesh;

	VertexID m_nMaxBaseVID;
	TriangleID m_nMaxBaseTID;
	VertexID m_nMaxMergeVID;
	TriangleID m_nMaxMergeTID;
	VertexID m_nMaxVID;
	TriangleID m_nMaxTID;
	void UpdateMaxIDs();

public:
	// !!! These should not really be public....hackhackhack !!!!

	inline bool IsMergeVID( VertexID vID ) const { return vID >= m_nMaxBaseVID; }
	inline VertexID ToMergeVID( VertexID vID ) const { return vID - m_nMaxBaseVID; }
	inline VertexID FromMergeVID( VertexID vID ) const {  return m_nMaxBaseVID + vID; }
	inline TriangleID ToMergeTID( TriangleID tID ) const { return tID - m_nMaxBaseTID; }
	inline TriangleID FromMergeTID( TriangleID tID ) const {  return m_nMaxBaseTID + tID; }

	SparseArray< VertexID > m_vBaseToMerge;
	SparseArray< VertexID > m_vMergeToBase;

protected:

/*
 * IMesh iterator interface
 */
	inline virtual void * ivtx_make_iterator(bool bStart) const
		{ return new vertex_iterator( (bStart) ? const_cast<VFMeshMerge*>(this)->begin_vertices() : const_cast<VFMeshMerge*>(this)->end_vertices() ); }
	inline virtual void * ivtx_make_iterator( void * pFromItr ) const
		{ return new vertex_iterator( * ((vertex_iterator *)pFromItr) ); }
	inline virtual void ivtx_free_iterator( void * pItr )  const
		{ delete (vertex_iterator *)pItr; }
	inline virtual void ivtx_set( void * pItr, void * pTo ) const
		{ *((vertex_iterator *)pItr) = *((vertex_iterator *)pTo); }
	inline virtual void ivtx_goto_next( void * pItr ) const
		{ ++(*((vertex_iterator *)pItr)); }
	inline virtual bool ivtx_equal( void * pItr1, void * pItr2 ) const
		{ return *((vertex_iterator *)pItr1) == *((vertex_iterator *)pItr2); }
	inline virtual VertexID ivtx_value( void * pItr ) const
		{ return **((vertex_iterator *)pItr); }

	inline virtual void * itri_make_iterator(bool bStart) const
		{ return new triangle_iterator( (bStart) ? const_cast<VFMeshMerge*>(this)->begin_triangles() : const_cast<VFMeshMerge*>(this)->end_triangles() ); }
	inline virtual void * itri_make_iterator( void * pFromItr ) const 
		{ return new triangle_iterator( * ((triangle_iterator *)pFromItr) ); }
	inline virtual void itri_free_iterator( void * pItr ) const 
		{ delete (triangle_iterator *)pItr; }
	inline virtual void itri_set( void * pItr, void * pTo ) const 
		{ *((triangle_iterator *)pItr) = *((triangle_iterator *)pTo); }
	inline virtual void itri_goto_next( void * pItr ) const 
		{ ++(*((triangle_iterator *)pItr)); }
	inline virtual bool itri_equal( void * pItr1, void * pItr2 ) const
		{ return *((triangle_iterator *)pItr1) == *((triangle_iterator *)pItr2); }
	inline virtual TriangleID itri_value( void * pItr ) const
		{ return **((triangle_iterator *)pItr); }
};





}  // end namespace rms

#endif // __LIBGEOMETRY_VF_MESH_MERGE_H__