// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#pragma once
#include "config.h"
#include <VFTriangleMesh.h>
#include <IMeshBVTree.h>
#include <Frame.h>

namespace rms {

class VertexSelection
{
public:
	VertexSelection(void);
	~VertexSelection(void);

	void SetSurface(rms::VFTriangleMesh * pMesh, rms::IMeshBVTree * pBVTree);

	void TryAddVertex( const rms::Frame3f & vFrame, bool bAppendToSelection = false );
	void ClearSelection();
	
	bool IsValid();
	const std::set< IMesh::VertexID > & Vertices() { return m_vVertices; }

	Wml::Vector3f Centroid();

	void Render(  );

	void SaveSelection(const char * pFilename);
	void LoadSelection(const char * pFilename);

protected:
	rms::VFTriangleMesh * m_pMesh;
	rms::IMeshBVTree * m_pBVTree;

	std::set< IMesh::VertexID > m_vVertices;
};


} // end namespace rms