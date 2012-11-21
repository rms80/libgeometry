// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#include "opengl.h"
#include "IMeshRenderer.h"


using namespace rms;

IMeshRenderer::IMeshRenderer( IMesh * pMesh, MeshPolygons * pPolygons )
{
	m_pMesh = pMesh;
	m_pPolygons = pPolygons;

	m_eNormalMode = FaceNormals;
	m_eColorMode = NoColors;
	m_eTexture2DMode = NoTexture2D;
	m_eEdgeMode = NoEdges;

	m_bDrawNormals = false;
	m_bDrawBoundaryEdges = false;
	m_bForcePolygonOffset = false;
}

IMeshRenderer::~IMeshRenderer(void)
{
}

void IMeshRenderer::Render()
{
	rms::IMesh::VertexID vTri[3];
	Wml::Vector3f vVtx[3], vNorm[3];
	Wml::Vector2f vTriUV[3];
	Wml::Vector2f vInvalidUV[3];
	vInvalidUV[0] = vInvalidUV[1] = vInvalidUV[2] = Wml::Vector2f(99999.0f, 99999.0f);
	Wml::Vector2f * pCurUV = vInvalidUV;
	Wml::Vector3f vFaceNormal;
//	Wml::ColorRGBA cColor;

	rms::IMesh::ITriIterator
		cur( m_pMesh->BeginITriangles() ),
		end( m_pMesh->EndITriangles() );

	if ( m_eEdgeMode != NoEdges || m_bForcePolygonOffset ) {
		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(1.0f,1.0f);
	}

	bool bHasUVSet = ( m_pMesh->HasUVSet(0) );

	glBegin(GL_TRIANGLES);
	while ( cur != end ) {
		rms::IMesh::TriangleID nTriID = *cur;	cur++;

		m_pMesh->GetTriangle( nTriID, vTri );
		m_pMesh->GetTriangle( nTriID, vVtx, vNorm );

		if ( m_eTexture2DMode == VertexTexture2D && bHasUVSet ) {
			pCurUV = ( m_pMesh->GetTriangleUV( nTriID, 0, vTriUV )) ? vTriUV : vInvalidUV;
		}

		if ( m_eNormalMode == FaceNormals ) {
			Wml::Vector3f vEdge1(vVtx[1] - vVtx[0]);		vEdge1.Normalize();
			Wml::Vector3f vEdge2(vVtx[2] - vVtx[0]);		vEdge2.Normalize();
			vFaceNormal = vEdge1.Cross(vEdge2);				vFaceNormal.Normalize();
			glNormal3fv(vFaceNormal);
		}

		for ( int j = 0 ; j < 3; ++j ) {
			if ( m_eNormalMode == VertexNormals )
				glNormal3fv( vNorm[j] );
			//if ( m_eColorMode == VertexColors ) {
			//	m_pMesh->GetColor( vTri[j], cColor );
			//	glColor4fv( cColor );
			//}
			if ( m_eTexture2DMode == VertexTexture2D )
				glTexCoord2fv( pCurUV[j] );
			glVertex3fv( vVtx[j] );
		}

	}
	glEnd();

	if ( m_eEdgeMode != NoEdges || m_bForcePolygonOffset) {
		glDisable(GL_POLYGON_OFFSET_FILL);
		Render_Wire();
	}

	if ( m_bDrawNormals )
		RenderNormals();
	if ( m_bDrawBoundaryEdges )
		DrawBoundaryEdges();
}



void IMeshRenderer::Render_Wire()
{
	glColor3f(0.0f, 0.0f, 0.0f);
	glPushAttrib(GL_ENABLE_BIT | GL_POLYGON_BIT | GL_LINE_BIT);
	glDisable(GL_LIGHTING);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	switch ( m_eEdgeMode ) {
		case AllEdges: {
			Wml::Vector3f vVtx[3];
			IMesh::ITriIterator
				cur( m_pMesh->BeginITriangles() ), end( m_pMesh->EndITriangles() );
			glBegin(GL_TRIANGLES);
			while ( cur != end ) {
				IMesh::TriangleID nTriID = *cur;	cur++;
				m_pMesh->GetTriangle( nTriID, vVtx );
				for ( int j = 0 ; j < 3; ++j )
					glVertex3fv( vVtx[j] );
			}
			glEnd();
		} break;

		case PolygonEdges: {
			if ( m_pPolygons == NULL ) {
				m_eEdgeMode = AllEdges;
				Render_Wire();
				m_eEdgeMode = PolygonEdges;
			} else {
				Wml::Vector3f vVertex, vTri[3];
				MeshPolygons::id_iterator curp(m_pPolygons->begin()), endp(m_pPolygons->end());
				while ( curp != endp ) {
					MeshPolygons::PolygonID sID = (*curp++);
					const std::vector<IMesh::VertexID> & vVerts = m_pPolygons->GetBoundary(sID);
					const std::vector<IMesh::TriangleID> & vTris = m_pPolygons->GetTriangles(sID);
					size_t nVCount = vVerts.size();
					size_t nTCount = vTris.size();
					if ( nVCount == 0 || nTCount == 0 )
						continue;

					bool bComplete = true;
					for ( unsigned int i = 0; bComplete && i < nTCount; ++i ) {
						if (! m_pMesh->IsTriangle(vTris[i]) )
							bComplete = false;
					};

					if ( bComplete ) {
						glBegin(GL_LINE_LOOP);
						for ( unsigned int k = 0; k < nVCount; ++k ) {
							m_pMesh->GetVertex(vVerts[k], vVertex);
							glVertex3fv( vVertex );
						}
						glEnd();
					} else {
						glBegin(GL_TRIANGLES);
						for ( unsigned int i = 0; i < nTCount; ++i ) {
							if ( m_pMesh->IsTriangle( vTris[i]) ) {
								m_pMesh->GetTriangle( vTris[i], vTri );
								for ( int j = 0 ; j < 3; ++j )
									glVertex3fv( vTri[j] );
							}
						}
						glEnd();
					}
				}
			}
		} break;

	}			  

	glPopAttrib();	
}




void IMeshRenderer::RenderNormals()
{
	float fScaleFactor = 0.025f;

	Wml::Vector3f vVtx[3], vNorm[3];

	glPushAttrib(GL_ENABLE_BIT | GL_LINE_BIT);
	glDisable(GL_LIGHTING);
	glLineWidth(2.0f);

	glColor4f(0.5f, 0.5f, 0.5f, 1.0f);

	glBegin(GL_LINES);
	rms::IMesh::ITriIterator
		cur( m_pMesh->BeginITriangles() ),
		end( m_pMesh->EndITriangles() );
	while ( cur != end ) {
		rms::IMesh::TriangleID nTriID = *cur;	cur++;
		m_pMesh->GetTriangle( nTriID, vVtx, vNorm );

		for ( int j = 0 ; j < 3; ++j ) {
			glVertex3fv( vVtx[j] );
			glVertex3fv( vVtx[j] + fScaleFactor * vNorm[j] );
		}

	}
	glEnd();

	glPopAttrib();
}



void IMeshRenderer::DrawBoundaryEdges()
{
	Wml::Vector3f vVtx[3];
	IMesh::TriangleID nTri[3];
	bool bIsBoundary[3];

	glPushAttrib(GL_ENABLE_BIT | GL_LINE_BIT);
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
	glLineWidth(4.0f);

	glColor4f(1.0f,0.0f,1.0f,1.0f);

	glBegin(GL_LINES);
	rms::IMesh::ITriIterator
		cur( m_pMesh->BeginITriangles() ),
		end( m_pMesh->EndITriangles() );
	while ( cur != end ) {
		rms::IMesh::TriangleID nTriID = *cur;	cur++;
		m_pMesh->GetTriangle( nTriID, vVtx );
		m_pMesh->GetTriangle( nTriID, nTri );

		for ( int j = 0 ; j < 3; ++j )
			bIsBoundary[j] = m_pMesh->IsBoundaryVertex( nTri[j] );

		for ( int j = 0 ; j < 3; ++j ) {
			if ( bIsBoundary[j] && bIsBoundary[(j+1)%3] ) {
				glVertex3fv( vVtx[j] );
				glVertex3fv( vVtx[(j+1)%3] );
			}
		}

	}
	glEnd();

	glPopAttrib();

}