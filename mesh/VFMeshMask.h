// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#ifndef __RMS_VF_MESH_MASK_H__
#define __RMS_VF_MESH_MASK_H__
#include "config.h"

#include "VFTriangleMesh.h"
#include "IMeshUVBVTree.h"
#include <WmlPolygon2.h>
#include <MeshSelection.h>
#include <MeshPolygons.h>
#include <ISurfaceProjector.h>


namespace rms {

class VFMeshMask : public IMesh, public ISurfaceProjector
{
public:
	VFMeshMask( );
	VFMeshMask( IMesh * pMesh );

	void SetMesh( IMesh * pMesh );
	void InitializeUVMask( IMesh * pMesh, IMesh::UVSetID nUVSetID = 0 );
	void InitializeUVMask(  IMesh * pMesh, rms::Polygon2f & vUVClipPolygon, IMesh::UVSetID nUVSetID = 0, bool bFilterConnectedComponents = false);

	// create a new mask that is the union of 2 existing masks
	void InitializeUnion( IMesh * pMesh, VFMeshMask & mask1, VFMeshMask & mask2 );

	void Initialize( const MeshSelection & triSelection );

	void ExpandToPolygons( const MeshPolygons & polygons );

	IMesh * GetMesh() { return m_pMesh; }

	enum MaskMode {
		Intersection, 
		Difference
	};
	void SetMaskMode( MaskMode eMode )
		{ m_eMaskMode = eMode; }
	MaskMode GetMaskMode() 
		{ return m_eMaskMode; }

	void Clear( bool bFreeMem = false ) 
		{ m_vCutVertices.clear(bFreeMem); m_vCutTriangles.clear(bFreeMem); if (HasUVSet(0)) ClearUVSet(0); m_bvTree.Clear(); }

	inline void SetCutVtx( IMesh::VertexID vID ) 
		{ m_vCutVertices.set( vID, true ); }

	inline bool IsCutVtx( IMesh::VertexID vID ) const
		{ return m_vCutVertices.has(vID); }

	inline void SetCutTri( IMesh::TriangleID tID ) 
		{ m_vCutTriangles.set( tID, true ); }

	inline bool IsCutTri( IMesh::TriangleID tID ) const
		{ return m_vCutTriangles.has(tID); }

	bool IsVisibleTri( IMesh::TriangleID tID );


/*
 * 2D/3D UV mapping
 * RMS TODO: these should really be in some kind of UV utility class...
 */
	Wml::Vector3f Project3D( const Wml::Vector2f & vUV, Wml::Vector3f * pNormal = NULL, bool * bStatus = NULL );
	Wml::Vector2f Project2D( const Wml::Vector3f & v3D, bool * bStatus = NULL );
	
	/*
	 * ISurfaceProjector interface
	 */
	virtual Wml::Vector2f ProjectToUV( const Wml::Vector3f & v3D, bool * bStatus = NULL ) 
		{ return Project2D(v3D, bStatus); }
	virtual Wml::Vector3f ProjectTo3D( const Wml::Vector2f & vUV, Wml::Vector3f * pNormal = NULL, bool * bStatus = NULL )
		{ return Project3D(vUV, pNormal, bStatus); }

/*
 * IMesh read interface (mandatory)
 */
	virtual void GetVertex( VertexID vID, Wml::Vector3f & vVertex, Wml::Vector3f * pNormal = NULL ) const 
		{ m_pMesh->GetVertex(vID, vVertex, pNormal); }
	virtual void GetNormal( VertexID vID, Wml::Vector3f & vNormal ) const
		{ m_pMesh->GetNormal(vID, vNormal); }
	virtual unsigned int GetVertexCount() const
		{ return m_pMesh->GetVertexCount(); }
	virtual unsigned int GetMaxVertexID() const
		{ return m_pMesh->GetMaxVertexID(); }
	virtual bool IsVertex( VertexID vID ) const
		{ return m_pMesh->IsVertex(vID) && ( (m_eMaskMode == Intersection) ?  IsCutVtx(vID) : !IsCutVtx(vID) ); }

	virtual void GetTriangle( TriangleID tID, VertexID vTriangle[3]  ) const
		{ m_pMesh->GetTriangle(tID, vTriangle); }
	virtual void GetTriangle( TriangleID tID, Wml::Vector3f vTriangle[3], Wml::Vector3f * pNormals = NULL ) const
		{ m_pMesh->GetTriangle(tID, vTriangle, pNormals); }
	virtual unsigned int GetTriangleCount() const
		{ return m_pMesh->GetTriangleCount(); }
	virtual unsigned int GetMaxTriangleID() const
		{ return m_pMesh->GetMaxTriangleID(); }
	virtual bool IsTriangle( TriangleID tID ) const
		{ return m_pMesh->IsTriangle(tID) && ( (m_eMaskMode == Intersection) ?  IsCutTri(tID) : !IsCutTri(tID) ); }


/*
 * IMesh mesh info interface - has default implementation
 */
	//! initialize vertex neighbour iteration
	virtual void BeginVtxTriangles( VtxNbrItr & v ) const;

	//! (possibly) un-ordered iteration around one-ring of a vertex. Returns InvalidID when done
	virtual TriangleID GetNextVtxTriangle( VtxNbrItr & v ) const;

	//! determine if a vertex is on the mesh boundary (if there is one)
	virtual bool IsBoundaryVertex( VertexID vID ) const;

	//! iterate around neighbour triangles of vID, sending each to callback
	virtual void NeighbourIteration( VertexID vID, NeighborTriCallback * pCallback );

/*
 * basic iterators
 */
	class vertex_iterator {
	public:
		inline vertex_iterator( const vertex_iterator & i2 ) 
			: m_pMesh(i2.m_pMesh),		m_isect_itr(i2.m_isect_itr),
			m_diff_cur(i2.m_diff_cur),  m_diff_end(i2.m_diff_end)
			{ }

		inline const vertex_iterator & operator=( const vertex_iterator & i2 ) { 
				m_pMesh = i2.m_pMesh;			m_isect_itr = i2.m_isect_itr;
				m_diff_cur = i2.m_diff_cur;		m_diff_end = i2.m_diff_end;
				return *this;
			}
		
		inline VertexID operator*() { 
			return ( m_pMesh->GetMaskMode() == VFMeshMask::Intersection ) ?
				m_isect_itr.index() : *m_diff_cur; }

		inline void operator++(int) { // postfix  (no return to avoid copies)
			goto_next(); }
		inline vertex_iterator & operator++() { // prefix
			goto_next();  return *this; }

		inline bool operator==( const vertex_iterator & i2 ) { 
			if ( m_pMesh->GetMaskMode() == VFMeshMask::Intersection )
				return m_isect_itr == i2.m_isect_itr;
			else
				return m_diff_cur == i2.m_diff_cur;
		}
		inline bool operator!=( const vertex_iterator & i2 ) { 
			return ! ( *this == i2 ); }

	protected:
		VFMeshMask * m_pMesh;
		SparseArray<bool>::iterator m_isect_itr;
		IMesh::IVtxIterator m_diff_cur;
		IMesh::IVtxIterator m_diff_end;

		inline void goto_next() {
			if ( m_pMesh->GetMaskMode() == VFMeshMask::Intersection )
				++m_isect_itr;
			else {
				++m_diff_cur;
				while ( m_diff_cur != m_diff_end && m_pMesh->IsCutVtx(*m_diff_cur)
					     && ! m_pMesh->IsBoundaryVertex(*m_diff_cur) )
					++m_diff_cur;
			}
		}
	
		//! if bStart, return start iterator, else return end iterator
		inline vertex_iterator( VFMeshMask * pMesh, bool bStart ) 
			: m_pMesh(pMesh), 
			m_diff_cur( (bStart) ? pMesh->m_pMesh->BeginIVertices() : pMesh->m_pMesh->EndIVertices()), 
			m_diff_end(pMesh->m_pMesh->EndIVertices()),
			m_isect_itr( (bStart) ? pMesh->m_vCutVertices.begin() : pMesh->m_vCutVertices.end() )
			{ while ( m_diff_cur != m_diff_end && m_pMesh->IsCutVtx(*m_diff_cur) 
					  && ! m_pMesh->IsBoundaryVertex(*m_diff_cur) )
				++m_diff_cur; }

		friend class VFMeshMask;		
	};
	friend class vertex_iterator;


	class triangle_iterator {
	public:
		inline triangle_iterator( const triangle_iterator & i2 ) 
			: m_pMesh(i2.m_pMesh),		m_isect_itr(i2.m_isect_itr),
			m_diff_cur(i2.m_diff_cur),  m_diff_end(i2.m_diff_end)
			{ }

		inline const triangle_iterator & operator=( const triangle_iterator & i2 ) { 
				m_pMesh = i2.m_pMesh;			m_isect_itr = i2.m_isect_itr;
				m_diff_cur = i2.m_diff_cur;		m_diff_end = i2.m_diff_end;
				return *this;
			}
		
		inline TriangleID operator*() { 
			return ( m_pMesh->GetMaskMode() == VFMeshMask::Intersection ) ?
				m_isect_itr.index() : *m_diff_cur; }

		inline void operator++(int) { // postfix  (no return to avoid copies)
			goto_next(); }
		inline triangle_iterator & operator++() { // prefix
			goto_next();  return *this; }

		inline bool operator==( const triangle_iterator & i2 ) { 
			if ( m_pMesh->GetMaskMode() == VFMeshMask::Intersection )
				return m_isect_itr == i2.m_isect_itr;
			else
				return m_diff_cur == i2.m_diff_cur;
		}
		inline bool operator!=( const triangle_iterator & i2 ) { 
			return ! ( *this == i2 ); }

	protected:
		VFMeshMask * m_pMesh;
		SparseArray<bool>::iterator m_isect_itr;
		IMesh::ITriIterator m_diff_cur;
		IMesh::ITriIterator m_diff_end;

		inline void goto_next() {
			if ( m_pMesh->GetMaskMode() == VFMeshMask::Intersection )
				++m_isect_itr;
			else {
				++m_diff_cur;
				while ( m_diff_cur != m_diff_end && m_pMesh->IsCutTri(*m_diff_cur) )
					++m_diff_cur;
			}
		}
	
		//! if bStart, return start iterator, else return end iterator
		inline triangle_iterator( VFMeshMask * pMesh, bool bStart ) 
			: m_pMesh(pMesh), 
			m_diff_cur( (bStart) ? pMesh->m_pMesh->BeginITriangles() : pMesh->m_pMesh->EndITriangles()), 
			m_diff_end(pMesh->m_pMesh->EndITriangles()),
			m_isect_itr( (bStart) ? pMesh->m_vCutTriangles.begin() : pMesh->m_vCutTriangles.end() )
			{ while ( m_diff_cur != m_diff_end && m_pMesh->IsCutTri(*m_diff_cur) )
				++m_diff_cur; }

		friend class VFMeshMask;		
	};
	friend class triangle_iterator;


	vertex_iterator begin_vertices()  { return vertex_iterator(this, true); }
	vertex_iterator end_vertices()  { return vertex_iterator(this, false); }

	triangle_iterator begin_triangles()  { return triangle_iterator(this, true); }
	triangle_iterator end_triangles()  { return triangle_iterator(this, false); }


protected:
	IMesh * m_pMesh;
	MaskMode m_eMaskMode;

	unsigned int m_nVertexSize;
	unsigned int m_nTriSize;

	// these should be changed to SparseFlagArray (when that class gets written)
	SparseArray<bool> m_vCutVertices;
	SparseArray<bool> m_vCutTriangles;

	//! BV tree for fast projection routines
	IMeshUVBVTree m_bvTree;

/*
 * IMesh iterator interface
 */
	inline virtual void * ivtx_make_iterator(bool bStart) const
		{ return new vertex_iterator( (bStart) ? const_cast<VFMeshMask *>(this)->begin_vertices() : const_cast<VFMeshMask *>(this)->end_vertices() ); }
	inline virtual void * ivtx_make_iterator( void * pFromItr ) const
		{ return new vertex_iterator( * ((vertex_iterator *)pFromItr) ); }
	inline virtual void ivtx_free_iterator( void * pItr ) const 
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
		{ return new triangle_iterator( (bStart) ? const_cast<VFMeshMask *>(this)->begin_triangles() : const_cast<VFMeshMask *>(this)->end_triangles() ); }
	inline virtual void * itri_make_iterator( void * pFromItr ) const
		{ return new triangle_iterator( * ((triangle_iterator *)pFromItr) ); }
	inline virtual void itri_free_iterator( void * pItr )  const
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





}  // end namespace rmsmesh

#endif // __RMS_VF_MESH_MASK_H__