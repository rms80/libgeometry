// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#ifndef __LIBGEOMETRY_GSURFACE_H__
#define __LIBGEOMETRY_GSURFACE_H__

#include "config.h"
#include "IMesh.h"
#include "VFTriangleMesh.h"
#include "MeshPolygons.h"
#include "IMeshBVTree.h"

namespace rms {

/*
 * Generic class for surfaces (trimesh, polygons, soup, point-cloud, etc)
 *  [TODO] support those other guys...
 */

class UVList : public DynamicVector<Wml::Vector2f>
{
public:
	UVList() {};
	UVList( unsigned int nSize );
};

class NormalList : public DynamicVector<Wml::Vector3f> 
{
public:
	NormalList() {};
	NormalList( unsigned int nSize );
};


class GSurface 
{
public:
	GSurface();
	GSurface(const GSurface & copy);
	GSurface(VFTriangleMesh * pMesh, MeshPolygons * pPolygons, bool bOwnsData = false);
	virtual ~GSurface();

	const GSurface & operator=(const GSurface & copy);
	void Copy( const VFTriangleMesh & mesh, const MeshPolygons & polygons);

	VFTriangleMesh & Mesh() { return *m_pMesh; }
	const VFTriangleMesh & Mesh() const { return *m_pMesh; }

	MeshPolygons & Polygons() { return *m_pPolygons; }
	const MeshPolygons & Polygons() const { return *m_pPolygons; }

	IMeshBVTree & BVTree() { return m_bvTree; }
	const IMeshBVTree & BVTree() const { return m_bvTree; }

	UVList & UV() { return m_vUVs; }
	const UVList & UV() const { return m_vUVs; }

	NormalList & Normals() { return m_vNormals; }
	const NormalList & Normals() const { return m_vNormals; }

	void Compact();


protected:
	VFTriangleMesh * m_pMesh;
	MeshPolygons * m_pPolygons;

	IMeshBVTree m_bvTree;

	UVList m_vUVs;
	NormalList m_vNormals;

	bool m_bOwnsData;
	void ResetToOwnedData();
};


} // namespace rms


#endif  // __LIBGEOMETRY_GSURFACE_H__