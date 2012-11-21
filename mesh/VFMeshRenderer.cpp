// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#include "opengl.h"
#include "VFMeshRenderer.h"

#include <rmsdebug.h>

using namespace rms;


VFMeshRenderer::VFMeshRenderer( const VFTriangleMesh * pMesh, const MeshPolygons * pPolygons )
{
	m_eNormalMode = FaceNormals;
	m_eColorMode = NoColors;
	m_eTexture2DMode = NoTexture2D;
	m_eColorTransferMode = ScaledToUnit;
	m_eEdgeMode = NoEdges;
	
	m_bDrawNormals = false;
	m_bDrawBoundary = false;
	m_bDrawNonManifoldEdges = false;
	
	m_bEnableScalarColor = false;
	m_nScalarColorSet = 0;

	m_bForcePolygonOffset = false;

	m_cScalarColorMin = Wml::ColorRGBA(1,1,1,1);
	m_cScalarColorMax = Wml::ColorRGBA(1,0,0,1);
	m_fScalarColorRange[0] = 0;
	m_fScalarColorRange[1] = 1;

	m_nGLDisplayList = -1;
	m_bGLDisplayListValid = false;
	m_bUseDisplayList = false;

	SetMesh(pMesh, pPolygons);
}

VFMeshRenderer::~VFMeshRenderer(void)
{
	if ( m_nGLDisplayList != -1 )
		glDeleteLists( m_nGLDisplayList, 1 );
}

void VFMeshRenderer::SetMesh( const VFTriangleMesh * pMesh, const MeshPolygons * pPolygons )
{
	m_pMesh = pMesh;
	m_pPolygons = pPolygons;
	m_pDrawFaces = NULL;

	m_bGLDisplayListValid = false;
}


void VFMeshRenderer::SetFaceFilterSet( const BitSet * pFaces, bool bInvert )
{
	m_pDrawFaces = pFaces;
	m_bInvertDrawFacesBits = bInvert;
	m_bGLDisplayListValid = false;
}


void VFMeshRenderer::Render()
{
	if ( m_bUseDisplayList && m_bGLDisplayListValid ) {
		glCallList(m_nGLDisplayList);
		return;
	}

	if ( m_bUseDisplayList ) {
		if ( m_nGLDisplayList == -1 )
			m_nGLDisplayList = glGenLists(1);
		glNewList(m_nGLDisplayList, GL_COMPILE_AND_EXECUTE);
	}

	Render_Mesh();

	if ( m_eEdgeMode != NoEdges )
		Render_Wire();

	if ( m_bDrawNormals )
		RenderNormals();

	if ( m_bDrawBoundary )
		DrawBoundaryEdges();

	if ( m_bDrawNonManifoldEdges )
		DrawNonManifoldEdges();

	if ( m_bUseDisplayList ) {
		glEndList();
		m_bGLDisplayListValid = true;
	}

}

void VFMeshRenderer::Render_Mesh()
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

	VFTriangleMesh::triangle_iterator 
		cur( m_pMesh->BeginTriangles() ),
		end( m_pMesh->EndTriangles() );

	bool bShowScalars = false;
	if ( m_bEnableScalarColor && m_pMesh->HasScalarSet(m_nScalarColorSet) )
		bShowScalars = true;

	// find min/max color values
	float fScalarMin = 99999.0f, fScalarMax = -99999.0f;
	if ( bShowScalars && m_eColorMode != Unscaled && m_eColorMode != ScaledToUnit_Custom ) {
		VFTriangleMesh::vertex_iterator curv(m_pMesh->BeginVertices()), endv(m_pMesh->EndVertices());
		while ( curv != endv ) {
			float fScalar;
			m_pMesh->GetScalar( *curv++, m_nScalarColorSet, fScalar );
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
	bool bPolygonOffsetEnabled = false;
	if ( m_eEdgeMode != NoEdges || m_bDrawNonManifoldEdges || m_bForcePolygonOffset ) {
		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(1.0f,1.0f);
		bPolygonOffsetEnabled = true;
	}



	glBegin(GL_TRIANGLES);
	while ( cur != end ) {
		IMesh::TriangleID nTriID = *cur;	cur++;

		if ( m_pDrawFaces ) {
			bool bIsVisible = m_pDrawFaces->get(nTriID);
			if ( (m_bInvertDrawFacesBits && bIsVisible ) || (!m_bInvertDrawFacesBits && !bIsVisible) )
				continue;
		}

		m_pMesh->GetTriangle( nTriID, vTri );
		m_pMesh->GetTriangle( nTriID, vVtx, vNorm );

		if ( m_eTexture2DMode == VertexTexture2D || m_eTexture2DMode == VertexTexture2D_Required ) {
			pCurUV = ( m_pMesh->GetTriangleUV( nTriID, 0, vTriUV )) ? vTriUV : vInvalidUV;
		}
		if ( m_eTexture2DMode == VertexTexture2D_Required && pCurUV == vInvalidUV )
			continue;

		if ( bShowScalars ) {
			bTriHasScalars = true;
			for ( int j = 0; j < 3; ++j )
				bTriHasScalars = bTriHasScalars && m_pMesh->GetScalar( vTri[j], m_nScalarColorSet, vTriScalar[j] );
		}

		for ( int j = 0 ; j < 3; ++j ) {
			if ( m_eNormalMode == VertexNormals )
				glNormal3fv( vNorm[j] );
			else if ( m_eNormalMode == FaceNormals ) {
				Wml::Vector3f vEdge1(vVtx[1] - vVtx[0]);			vEdge1.Normalize();
				Wml::Vector3f vEdge2(vVtx[2] - vVtx[0]);			vEdge2.Normalize();
				Wml::Vector3f vFaceNormal( vEdge1.Cross(vEdge2) );	vFaceNormal.Normalize();
				glNormal3fv(vFaceNormal);
			}

			if ( m_eColorMode == VertexColors ) {
				m_pMesh->GetColor( vTri[j], cColor );
				glColor4fv( cColor );
			}
			if ( m_eTexture2DMode == VertexTexture2D || m_eTexture2DMode == VertexTexture2D_Required )
				glTexCoord2fv( pCurUV[j] );

			if ( bShowScalars && bTriHasScalars ) {
				float fV = vTriScalar[j];

				switch ( m_eColorTransferMode ) {
					default:
					case Unscaled:
						vColor = fV*vColorA + (1-fV)*vColorB;
						glColor3fv(vColor);
						//glColor3f( cColor.R() * fV, cColor.G() * fV, cColor.B() * fV );
						break;
					case ScaledToUnit: {
						float fUnitV = (fV - fScalarMin) / (fScalarMax-fScalarMin);
						glColor3f( cColor.R() * fUnitV, cColor.G() * fUnitV, cColor.B() * fUnitV );
						//if ( fUnitV == 1.1f )
						//	glColor3f(0.6f, 0.6f, 1.0f);
						//else
						//	glColor3fv( (fUnitV)*Wml::Vector3f(1,0,0) + (1-fUnitV)*Wml::Vector3f(1,1,1) );
						} break;
					case ScaledToUnit_Custom: {
						if ( fV < m_fScalarColorRange[0] )	fV = m_fScalarColorRange[0];
						if ( fV > m_fScalarColorRange[1] )	fV = m_fScalarColorRange[1];
						float fUnitV = (fV - m_fScalarColorRange[0]) / (m_fScalarColorRange[1]-m_fScalarColorRange[0]);
						glColor3fv( (fUnitV)*m_cScalarColorMax + (1-fUnitV)*m_cScalarColorMin );
						} break;
					case SinModulate: {
						float fUnitV = (fV - fScalarMin) / (fScalarMax-fScalarMin);
						fV = (1.0f + sin(45.0f * fUnitV)) / 2.0f;
						//if ( fUnitV > 2.0f/5.0f && fUnitV < 5.0f/10.0f ) {
						//	Wml::Vector3f c1(1.0f, 0.0f, 0.0f);						Wml::Vector3f c2(0.0f, 0.0f, 0.0f);
						//	glColor3fv( (1-fV)*c2 + fV*c1 );
						//} else {
							glColor3f( cColor.R() * fV, cColor.G() * fV, cColor.B() * fV );
						//}
						} break;
				}
						


				// hack for per-triangle scalar renderering
				//if ( vTriScalar[0] == 1 || vTriScalar[1] == 1 || vTriScalar[2] == 1 )
				//	fV = 1;
				//else if ( vTriScalar[0] > 1 || vTriScalar[1] > 1 || vTriScalar[2] > 1 )
				//	fV = 1.1;
				//if ( fV == 1.0f )
				//	glColor3f( 1, 1, 1 );
				//else if ( fV > 1 )
				//	glColor3f( 0.8f, 0.8f, 1.0f );
				//else
				//	glColor3f( 1.0f, 1.0f-fV, 1.0f-fV);

				// fading color
				//fV = (1.0f + sin(1.0f * fV)) / 2.0f;
				//fV = (float)abs(sin(1.0f * fV));
				//glColor3f( cColor.R() * fV, cColor.G() * fV, cColor.B() * fV );


				// set color for bin
				//int nV = (int)fV;
				//if ( nV == 0 )
				//	glColor3f(0.0f, 0.0f, 0.0f);
				//else if ( nV % 3 == 0 )
				//	glColor3f(1.0f, 0.0f, 0.0f);
				//else if ( nV % 3 == 1 )
				//	glColor3f(0.0f, 1.0f, 0.0f);
				//else
				//	glColor3f(0.0f, 0.0f, 1.0f);

			}

			glVertex3fv( vVtx[j] );
		}

	}
	glEnd();

	if ( bPolygonOffsetEnabled )
		glDisable(GL_POLYGON_OFFSET_FILL);

	glPopAttrib();
}



void VFMeshRenderer::Render_Wire(const Wml::ColorRGBA & cColor)
{
	glColor4f(cColor.R(), cColor.G(), cColor.B(), cColor.A());
	glPushAttrib(GL_ENABLE_BIT | GL_POLYGON_BIT | GL_LINE_BIT);
	glDisable(GL_LIGHTING);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	//glLineWidth(2.0f);

	switch ( m_eEdgeMode ) {
		case AllEdges: {
			Wml::Vector3f vVtx[3];
			VFTriangleMesh::triangle_iterator 
				cur( m_pMesh->BeginTriangles() ), end( m_pMesh->EndTriangles() );

			glBegin(GL_TRIANGLES);
			while ( cur != end ) {
				IMesh::TriangleID nTriID = *cur;	cur++;

				if ( m_pDrawFaces ) {
					bool bIsVisible = m_pDrawFaces->get(nTriID);
					if ( (m_bInvertDrawFacesBits && bIsVisible ) || (!m_bInvertDrawFacesBits && !bIsVisible) )
						continue;
				}

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
				MeshPolygons::id_iterator curp(m_pPolygons->begin()), endp(m_pPolygons->end());
				while ( curp != endp ) {
					MeshPolygons::PolygonID sID = (*curp++);


					if ( m_pDrawFaces ) {
						const std::vector<IMesh::TriangleID> & vTris = m_pPolygons->GetTriangles(sID);
						bool bIsTriVisible = m_pDrawFaces->get(vTris[0]);
						bool bIsPolyVisible = (m_bInvertDrawFacesBits && !bIsTriVisible ) || (!m_bInvertDrawFacesBits && bIsTriVisible);
						size_t nCount = vTris.size();
						for ( unsigned int k = 1; bIsPolyVisible && k < nCount; ++k ) {
							bIsTriVisible = m_pDrawFaces->get(vTris[k]);
							bIsPolyVisible = (m_bInvertDrawFacesBits && !bIsTriVisible ) || (!m_bInvertDrawFacesBits && bIsTriVisible);
						}
						if ( ! bIsPolyVisible )
							continue;
					}

					const std::vector<IMesh::VertexID> & vVerts = m_pPolygons->GetBoundary(sID);
					size_t nCount = vVerts.size();
					glBegin(GL_LINE_LOOP);
					for ( unsigned int k = 0; k < nCount; ++k )
						glVertex3fv( m_pMesh->GetVertex(vVerts[k]) );
					glEnd();
				}
			}
		} break;

	}			  

	glPopAttrib();	
}


void VFMeshRenderer::RenderNormals()
{
	float fScaleFactor = 0.025f;

	Wml::Vector3f vVtx[3], vNorm[3];

	glPushAttrib(GL_ENABLE_BIT | GL_LINE_BIT);
	glDisable(GL_LIGHTING);
	glLineWidth(2.0f);

	glColor4f(0.5f, 0.5f, 0.5f, 1.0f);

	glBegin(GL_LINES);
	VFTriangleMesh::triangle_iterator 
		cur( m_pMesh->BeginTriangles() ),
		end( m_pMesh->EndTriangles() );
	while ( cur != end ) {
		IMesh::TriangleID nTriID = *cur;	cur++;

		if ( m_pDrawFaces && ! m_pDrawFaces->get(nTriID) )
			continue;

		m_pMesh->GetTriangle( nTriID, vVtx, vNorm );
		for ( int j = 0 ; j < 3; ++j ) {
			glVertex3fv( vVtx[j] );
			glVertex3fv( vVtx[j] + fScaleFactor * vNorm[j] );
		}

	}
	glEnd();

	glPopAttrib();
}





void VFMeshRenderer::Render_UV()
{
	Wml::Vector2f vTriUV[3];
	Wml::Vector2f vInvalidUV[3];
	vInvalidUV[0] = vInvalidUV[1] = vInvalidUV[2] = Wml::Vector2f(99999.0f, 99999.0f);
	Wml::Vector2f * pCurUV;

	VFTriangleMesh::triangle_iterator 
		cur( m_pMesh->BeginTriangles() ),
		end( m_pMesh->EndTriangles() );

	glBegin(GL_TRIANGLES);
	while ( cur != end ) {
		IMesh::TriangleID nTriID = *cur;	cur++;

		pCurUV = ( m_pMesh->GetTriangleUV( nTriID, 0, vTriUV )) ? vTriUV : vInvalidUV;
		if ( pCurUV == vInvalidUV )
			continue;

		for ( int j = 0 ; j < 3; ++j ) {
			glTexCoord2fv( pCurUV[j] );
			glVertex2fv( pCurUV[j] );
		}

	}
	glEnd();
}



void VFMeshRenderer::DrawBoundaryEdges()
{
	glPushAttrib(GL_ENABLE_BIT | GL_LINE_BIT);
	glDisable(GL_LIGHTING);
	glLineWidth(4.0f);

	glColor4f(0.0f,0.0f,1.0f,1.0f);

	glBegin(GL_LINES);

	rms::IMesh::TriangleID nNbr[3];
	Wml::Vector3f vTri[3];
	rms::VFTriangleMesh::triangle_iterator curt(m_pMesh->BeginTriangles()), endt(m_pMesh->EndTriangles());
	while ( curt != endt ) {
		rms::IMesh::TriangleID tID = *curt; ++curt;
		m_pMesh->FindNeighbours(tID, nNbr);
		m_pMesh->GetTriangle(tID, vTri);
		for ( int j = 0; j < 3; ++j ) {
			if ( nNbr[j] == rms::IMesh::InvalidID ) {
				glVertex3fv( vTri[j] );
				glVertex3fv( vTri[ (j+1) % 3 ] );
			}
		}
	}
	glEnd();

	glPopAttrib();
}



void VFMeshRenderer::DrawNonManifoldEdges()
{
	if (m_pMesh->IsManifold())
		return;

	glPushAttrib(GL_ENABLE_BIT | GL_LINE_BIT | GL_POINT_BIT);
	glDisable(GL_LIGHTING);
	glLineWidth(4.0f);
	glColor4f(1.0f,0.0f,0.0f,1.0f);

	glLineWidth(3.0f);
	glPointSize(5.0f);

	const std::set<IMesh::EdgeID> & vEdges = m_pMesh->NonManifoldEdges();
	std::set<IMesh::EdgeID>::const_iterator cure(m_pMesh->NonManifoldEdges().begin()), ende(m_pMesh->NonManifoldEdges().end());
	while ( cure != ende ) {
		IMesh::EdgeID eID = *cure++;
		Wml::Vector3f vEdge[2];
		m_pMesh->GetEdge(eID, vEdge);
		glBegin(GL_POINTS);
		glVertex3fv( vEdge[0] );
		glVertex3fv( vEdge[1] );
		glEnd();
		glBegin(GL_LINES);
		glVertex3fv( vEdge[0] );
		glVertex3fv( vEdge[1] );
		glEnd();
	}

	glPopAttrib();
}

