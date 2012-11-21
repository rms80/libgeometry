// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#ifndef __RMS_MESH_POLYGONS_H__
#define __RMS_MESH_POLYGONS_H__
#include "config.h"
#include <IMesh.h>
#include <VFTriangleMesh.h>
#include <IDMap.h>

namespace rms {


/*
 * Basically this class is used to track polygons on a triangle mesh. IE if
 * the mesh is a quad mesh, this class can be used to keep track of the pairs
 * of triangles in a VFTriangleMesh. In theory it could be used for other things.
 * But it might make sense to replace it with a half-edge overlay mesh which
 * is specific to the polymesh problem.
 *
 * Also it would be more efficient to represent the face groups as MaxTriangleID() array
 * of face IDs (although this will make it more expensive to get a list of faces...
 * maybe use a [faceID,pNext] list? and add an iterator?)
 */
class MeshPolygons
{
public:
	typedef unsigned int PolygonID;
	static unsigned int InvalidPolygonID;

	static const std::vector<IMesh::TriangleID> EMPTY_SET;
	static const std::vector<IMesh::VertexID> EMPTY_BOUNDARY;
	static const std::vector<unsigned int> EMPTY_BOUNDARY_UV;


	MeshPolygons();
	MeshPolygons(VFTriangleMesh & trimesh);
	MeshPolygons(VFTriangleMesh & newMesh, const MeshPolygons & oldSets, const TriangleMap & TMap, const VertexMap * VMap);

	void Clear();

	void Initialize(VFTriangleMesh & mesh, const MeshPolygons & oldSets, const TriangleMap & TMap, const VertexMap * VMap);

	PolygonID CreatePolygon(unsigned int nSizeHint = 0);
	bool AppendTriangle( PolygonID sID, IMesh::TriangleID tID );

	// [TODO] replace this with automatic boundary updates...
	bool SetBoundary( PolygonID sID, const std::vector<IMesh::VertexID> & vBoundary, const std::vector<unsigned int> * pBoundaryUV = NULL );

	//! input is data from mesh.Append(append, TMap) - generate faces for triangles in append
	void AppendTriangles( VFTriangleMesh & mesh, VFTriangleMesh & append, TriangleMap & TMap );

	//! input is data from mesh.Append(append, TMap, VMap) - copy/rewrite faces from appendpolys
	void AppendPolygons( VFTriangleMesh & mesh, VFTriangleMesh & append, MeshPolygons & appendpolys, 
						 VertexMap & VMap, TriangleMap & TMap, UVMap * pUVMap = NULL );

	//! add triangular polys for any unused triangles
	void TriangleFill(const VFTriangleMesh & mesh);


	void Rewrite( const VertexMap & VMap, const TriangleMap & TMap );
	void RewriteBoundaries( const VertexMap & VMap );
	void RewriteUVs( unsigned int nShift );


	PolygonID FindPolygon( IMesh::TriangleID tID );

	const std::vector<IMesh::TriangleID> & GetTriangles( PolygonID sID ) const;
	const std::vector<IMesh::VertexID> & GetBoundary( PolygonID sID ) const;
	const std::vector<unsigned int> & GetBoundaryUV( PolygonID sID ) const;

	void ClearUVs();

	void ToSets( const std::set<IMesh::TriangleID> & vTris, std::set<IMesh::TriangleID> & vSetTris ) const;

	unsigned int MaxPolygonSize() { return m_nMaxPolygonSize; }

	void SanityCheck(VFTriangleMesh & mesh);

protected:
	struct Polygon {
		PolygonID id;
		std::vector<IMesh::TriangleID> vTris;
		std::vector<IMesh::VertexID> vBoundary;
		std::vector<unsigned int> vBoundaryUV;
		Polygon() {
			id = InvalidPolygonID;  vTris.clear(); vBoundary.clear();
		}
		bool operator<( const Polygon & set2 ) const {
			return id < set2.id;
		}
	};
	std::set<Polygon> m_vPolygons;
	typedef std::set<Polygon>::iterator PolygonItr;
	typedef std::set<Polygon>::const_iterator PolygonConstItr;

	std::map<IMesh::TriangleID, PolygonID> m_PolygonMap; 

	unsigned int m_nMaxPolygonSize;

	unsigned int m_nIDCounter;

public:
	
	class id_iterator {
	public:
		id_iterator(const id_iterator & copy) { cur = copy.cur; }

		inline MeshPolygons::PolygonID operator*() { 
			return (*cur).id;
		}

		inline id_iterator & operator++() {		// prefix
			++cur;
			return *this;
		}
		inline id_iterator operator++(int) {		// postfix
			id_iterator copy(*this);
			++cur;
			return copy;
		}

		inline bool operator==( id_iterator & r2 ) {
			return cur == r2.cur;
		}
		inline bool operator!=( id_iterator & r2 ) {
			return cur != r2.cur;
		}		

	protected:
		PolygonConstItr cur;

		id_iterator( PolygonConstItr & val ) { cur = val; }
		friend class MeshPolygons;
	};

	id_iterator begin() const { return id_iterator(m_vPolygons.begin()); }
	id_iterator end() const { return id_iterator(m_vPolygons.end()); }

};




} // end namespace rms


#endif // __RMS_MESH_POLYGONS_H__