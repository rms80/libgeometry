// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#include "opengl.h"
#include "VertexSelection.h"

#include <limits>
#include <Wm4DistVector3Segment3.h>

#include <MeshUtils.h>

using namespace rms;

VertexSelection::VertexSelection(void)
{
	m_pMesh = NULL;
	m_pBVTree = NULL;
}

VertexSelection::~VertexSelection(void)
{
}

void VertexSelection::SetSurface(rms::VFTriangleMesh * pMesh, rms::IMeshBVTree * pBVTree)
{
	m_pMesh = pMesh;
	m_pBVTree = pBVTree;
}

void VertexSelection::TryAddVertex( const rms::Frame3f & vFrame, bool bAppendToSelection )
{
	rms::IMesh::VertexID vID;
	if ( m_pBVTree->FindNearestVtx( vFrame.Origin(), vID ) )
		m_vVertices.insert(vID);
}
void VertexSelection::ClearSelection()
{
	m_vVertices.clear();
}



bool VertexSelection::IsValid()
{
	return ! m_vVertices.empty();
}


Wml::Vector3f VertexSelection::Centroid()
{
	Wml::Vector3f v, vCentroid( Wml::Vector3f::ZERO );
	int nCount = 0;
	std::set< IMesh::VertexID >::iterator curv(m_vVertices.begin()), endv(m_vVertices.end());
	while ( curv != endv ) {
		m_pMesh->GetVertex( *curv++, v );
		vCentroid += v;
		++nCount;
	}
	return vCentroid / (float)nCount;
}




void VertexSelection::Render( )
{
	glPushAttrib(GL_ENABLE_BIT | GL_LINE_BIT | GL_POINT_BIT);
	glDisable(GL_LIGHTING);
	glDepthFunc(GL_LEQUAL);
	
	static const float fNormOff = 0.001f;

	// render selection verts
	glPointSize(5.0f);
	glBegin(GL_POINTS);
	glColor3f(0,0,1);
	std::set< IMesh::VertexID >::iterator curv(m_vVertices.begin()), endv(m_vVertices.end());
	while ( curv != endv ) {
		Wml::Vector3f v;
		m_pMesh->GetVertex( *curv++, v );
		glVertex3fv( v );
	}
	glEnd();


	glPopAttrib();	
}




void VertexSelection::SaveSelection(const char * pFilename)
{
	std::ofstream out(pFilename);
	if ( ! out )
		return;
	std::set< IMesh::VertexID >::iterator curv(m_vVertices.begin()), endv(m_vVertices.end());
	while ( curv != endv )
		out << *curv++ << std::endl;
	out.close();
}

void VertexSelection::LoadSelection(const char * pFilename)
{
	std::ifstream in(pFilename);
	if ( ! in )
		return;

	while ( in ) {
		int nVtxID;
		in >> nVtxID;
		m_vVertices.insert(nVtxID);
	}
}




