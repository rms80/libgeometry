// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#include "opengl.h"
#include "GSurfaceRenderer.h"

using namespace rms;

GSurfaceRenderer::GSurfaceRenderer( GSurface * pSurface )
{
	m_pSurface = pSurface;

	m_eNormalMode = FaceNormals;
	m_eColorMode = NoColors;
	m_eTexture2DMode = NoTexture2D;
	m_eColorTransferMode = ScaledToUnit;
	m_eEdgeMode = NoEdges;
	
	m_bDrawNormals = false;
	
	m_bEnableScalarColor = false;
	m_nScalarColorSet = 0;

}

GSurfaceRenderer::~GSurfaceRenderer(void)
{
}

void GSurfaceRenderer::Render()
{
	IMesh::VertexID vTri[3];
	Wml::Vector3f vVtx[3], vNorm[3];
	Wml::Vector2f vTriUV[3];
	Wml::Vector2f vInvalidUV[3];
	vInvalidUV[0] = vInvalidUV[1] = vInvalidUV[2] = Wml::Vector2f(99999.0f, 99999.0f);
	float vTriScalar[3];
	bool bTriHasScalars;
	Wml::Vector2f * pCurUV;
	Wml::ColorRGBA cColor =  Wml::ColorRGBA::WHITE;
	Wml::Vector3f vColorA(1,0,0), vColorB(0.8f, 0.8f, 1.0f), vColor;

	VFTriangleMesh * pMesh = &m_pSurface->Mesh();

	VFTriangleMesh::triangle_iterator 
		cur( pMesh->BeginTriangles() ),
		end( pMesh->EndTriangles() );

	bool bShowScalars = false;
	if ( m_bEnableScalarColor && pMesh->HasScalarSet(m_nScalarColorSet) )
		bShowScalars = true;

	// find min/max color values
	float fScalarMin = 99999.0f, fScalarMax = -99999.0f;
	if ( bShowScalars ) {
		VFTriangleMesh::vertex_iterator curv(pMesh->BeginVertices()), endv(pMesh->EndVertices());
		while ( curv != endv ) {
			float fScalar;
			pMesh->GetScalar( *curv++, m_nScalarColorSet, fScalar );
			if ( fScalar < fScalarMin ) fScalarMin = fScalar;
			if ( fScalar > fScalarMax ) fScalarMax = fScalar;
		}
		if ( fScalarMin == fScalarMax ) 
			fScalarMax = fScalarMin + 1;
	}


	glPushAttrib( GL_ENABLE_BIT | GL_LIGHTING_BIT );

	//if ( bShowScalars )
	//	glDisable(GL_LIGHTING);
	if ( bShowScalars ) {
		float fSpec[4] = {0,0,0,0};
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, fSpec);
	}
	if ( m_eEdgeMode != NoEdges ) {
		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(1.0f,1.0f);
	}



	glBegin(GL_TRIANGLES);
	while ( cur != end ) {
		IMesh::TriangleID nTriID = *cur;	cur++;

		pMesh->GetTriangle( nTriID, vTri );
		pMesh->GetTriangle( nTriID, vVtx, vNorm );

		if ( m_eTexture2DMode == VertexTexture2D || m_eTexture2DMode == VertexTexture2D_Required ) {
			pCurUV = ( pMesh->GetTriangleUV( nTriID, 0, vTriUV )) ? vTriUV : vInvalidUV;
		}
		if ( m_eTexture2DMode == VertexTexture2D_Required && pCurUV == vInvalidUV )
			continue;

		if ( bShowScalars ) {
			bTriHasScalars = true;
			for ( int j = 0; j < 3; ++j )
				bTriHasScalars = bTriHasScalars && pMesh->GetScalar( vTri[j], m_nScalarColorSet, vTriScalar[j] );
		}

		for ( int j = 0 ; j < 3; ++j ) {
			if ( m_eNormalMode == VertexNormals )
				glNormal3fv( vNorm[j] );
			else if ( m_eNormalMode == FaceNormals ) {
				Wml::Vector3f vEdge1(vVtx[1] - vVtx[0]);				vEdge1.Normalize();
				Wml::Vector3f vEdge2(vVtx[2] - vVtx[0]);				vEdge2.Normalize();
				Wml::Vector3f vFaceNormal( vEdge1.Cross(vEdge2) );		vFaceNormal.Normalize();
				glNormal3fv(vFaceNormal);
			}

			if ( m_eColorMode == VertexColors ) {
				pMesh->GetColor( vTri[j], cColor );
				glColor4fv( cColor );
			}
			if ( m_eTexture2DMode == VertexTexture2D || m_eTexture2DMode == VertexTexture2D_Required )
				glTexCoord2fv( pCurUV[j] );

			if ( bShowScalars && bTriHasScalars ) {
				float fV = vTriScalar[j];

				float fUnitV = (fV - fScalarMin) / (fScalarMax-fScalarMin);
				switch ( m_eColorTransferMode ) {
					default:
					case Unscaled:
						vColor = fV*vColorA + (1-fV)*vColorB;
						glColor3fv(vColor);
						//glColor3f( cColor.R() * fV, cColor.G() * fV, cColor.B() * fV );
						break;
					case ScaledToUnit: {
						glColor3f( cColor.R() * fUnitV, cColor.G() * fUnitV, cColor.B() * fUnitV );
						//if ( fUnitV == 1.1f )
						//	glColor3f(0.6f, 0.6f, 1.0f);
						//else
						//	glColor3fv( (fUnitV)*Wml::Vector3f(1,0,0) + (1-fUnitV)*Wml::Vector3f(1,1,1) );
						} break;
					case SinModulate:
						fV = (1.0f + sin(45.0f * fUnitV)) / 2.0f;
						//if ( fUnitV > 2.0f/5.0f && fUnitV < 5.0f/10.0f ) {
						//	Wml::Vector3f c1(1.0f, 0.0f, 0.0f);						Wml::Vector3f c2(0.0f, 0.0f, 0.0f);
						//	glColor3fv( (1-fV)*c2 + fV*c1 );
						//} else {
							glColor3f( cColor.R() * fV, cColor.G() * fV, cColor.B() * fV );
						//}
						break;
				}

			}

			glVertex3fv( vVtx[j] );
		}

	}
	glEnd();

	if ( m_eEdgeMode != NoEdges ) {
		glDisable(GL_POLYGON_OFFSET_FILL);
		Render_Wire();
	}

	if ( m_bDrawNormals )
		RenderNormals();

	glPopAttrib();
}



void GSurfaceRenderer::Render_Wire()
{
	glColor3f(0.0f, 0.0f, 0.0f);
	glPushAttrib(GL_ENABLE_BIT | GL_POLYGON_BIT | GL_LINE_BIT);
	glDisable(GL_LIGHTING);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	switch ( m_eEdgeMode ) {
		case AllEdges: {
			MeshPolygons::id_iterator curp(m_pSurface->Polygons().begin()), endp(m_pSurface->Polygons().end());
			while ( curp != endp ) {
				MeshPolygons::PolygonID sID = (*curp++);
				const std::vector<IMesh::VertexID> & vVerts = m_pSurface->Polygons().GetBoundary(sID);
				size_t nCount = vVerts.size();
				glBegin(GL_LINE_LOOP);
				for ( unsigned int k = 0; k < nCount; ++k )
					glVertex3fv(m_pSurface->Mesh().GetVertex(vVerts[k]) );
				glEnd();
			}
		} break;

	}			  

	glPopAttrib();	
}


void GSurfaceRenderer::RenderNormals()
{
	float fScaleFactor = 0.025f;

	Wml::Vector3f vVtx[3], vNorm[3];

	glPushAttrib(GL_ENABLE_BIT | GL_LINE_BIT);
	glDisable(GL_LIGHTING);
	glLineWidth(2.0f);

	glColor4f(0.5f, 0.5f, 0.5f, 1.0f);

	glBegin(GL_LINES);
	VFTriangleMesh::triangle_iterator 
		cur( m_pSurface->Mesh().BeginTriangles() ),
		end( m_pSurface->Mesh().EndTriangles() );
	while ( cur != end ) {
		IMesh::TriangleID nTriID = *cur;	cur++;
		m_pSurface->Mesh().GetTriangle( nTriID, vVtx, vNorm );

		for ( int j = 0 ; j < 3; ++j ) {
			glVertex3fv( vVtx[j] );
			glVertex3fv( vVtx[j] + fScaleFactor * vNorm[j] );
		}

	}
	glEnd();

	glPopAttrib();
}





void GSurfaceRenderer::DrawBoundaryEdges()
{
	glPushAttrib(GL_ENABLE_BIT | GL_LINE_BIT);
	glDisable(GL_LIGHTING);
	glLineWidth(4.0f);

	glColor4f(1.0f,0.0f,0.0f,1.0f);

	glBegin(GL_LINES);

	Wml::Vector3f vEdge[2];
	rms::VFTriangleMesh::edge_iterator cure(m_pSurface->Mesh().BeginEdges()), ende(m_pSurface->Mesh().EndEdges());
	while ( cure != ende ) {
		rms::IMesh::EdgeID eID = *cure; ++cure;
		if ( m_pSurface->Mesh().IsBoundaryEdge(eID) ) {
			m_pSurface->Mesh().GetEdge(eID, vEdge);
			glVertex3fv( vEdge[0] );
			glVertex3fv( vEdge[1] );
		}
	}
	glEnd();

	glPopAttrib();
}





