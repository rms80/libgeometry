// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#include "config.h"
#include <VFTriangleMesh.h>
#include <ExpMapGenerator.h>

class MeshGeodesic
{
public:
	MeshGeodesic( void );
	MeshGeodesic( const MeshGeodesic & m2 );
	~MeshGeodesic(void);

	void SetSurface(rms::VFTriangleMesh * pMesh, rms::IMeshBVTree * pBVTree, rms::ExpMapGenerator * pExpgen);

	void SetPoint1( const rms::Frame3f & vPoint1 )
		{ m_vPoint1 = vPoint1; Invalidate(); }
	void SetPoint2( const rms::Frame3f & vPoint2 )
		{ m_vPoint2 = vPoint2; Invalidate(); }

	void SetSurfacePoints( const std::vector<rms::Frame3f> & vCurve );

	void Clear();

	void DoOptimizeStep() { OptimizePath(); }

	void Transport( const Wml::Vector2f & vDirection, const rms::Frame3f * pPrevFrame = NULL );	
	void GetEndpoints( rms::Frame3f & vEndPoint1, rms::Frame3f & vEndPoint2 );

	void Render(const Wml::ColorRGBA & cEdgeColor, bool bDrawDijkstraPath = true, bool bDrawEndpoints = true);

	float Distance( const Wml::Vector3f & vVertex, Wml::Vector3f & vNearest );

	const std::vector<rms::Frame3f> & GetPath();

protected:
	rms::VFTriangleMesh * m_pMesh;
	rms::IMeshBVTree * m_pBVTree;
	rms::ExpMapGenerator * m_pExpGen;

	bool m_bIsValid;
	void Validate();
	void Invalidate() { m_bIsValid = false; m_vDijkstraPath.resize(0); }

	rms::Frame3f m_vPoint1;
	rms::Frame3f m_vPoint2;

	std::vector< rms::Frame3f > m_vDijkstraPath;

	void OptimizePath();
	std::vector< rms::Frame3f > m_vOptPath;
	std::vector< Wml::Vector2f > m_vLocalUVs;
};
