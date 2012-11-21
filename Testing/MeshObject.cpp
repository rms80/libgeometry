// Ryan Schmidt - rms@unknownroad.com
// Copyright (c) 2006-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt


// #include "winstuff.h"
#include "MeshObject.h"
#include "rmsdebug.h"

#ifdef WIN_32
  #include <gl/GL.h>
  #include <gl/GLU.h>
#else
  #include <GL/gl.h>
  #include <GL/glu.h>
  #include <GL/glut.h>
#endif


#include <VFMeshRenderer.h>
#include <string>
#include <Wm4IntpThinPlateSpline2.h>
#include <Triangulator2D.h>
#include <VFMeshMask.h>

MeshObject::MeshObject(void)
{
	m_bExpMapValid = false;
	m_bComputeExpMap = true;
	m_fDecalRadius = 0.3f;

	// create checker texture...
	std::vector<unsigned char> vTexBuf;
	
	Texture::MakeCheckerImage(512, 64, vTexBuf, true);
	
	m_texture.InitializeBuffer(512, 512, vTexBuf, GL_RGBA);
	m_texture.SetTextureInfo( GL_TEXTURE_2D, GL_LINEAR, GL_REPEAT );
	m_texture.TextureEnvMode() = GL_DECAL;
	//m_texture.SetEdgeAlpha( 0 );
	
}

MeshObject::~MeshObject(void)
{
}

void MeshObject::ReadMeshOBJ( const char * pFilename )
{

	rms::VFTriangleMesh readMesh;
	std::string err;
	bool bOK = readMesh.ReadOBJ(pFilename,err);
	if ( ! bOK ) {
		fprintf(stderr, "Error opening file %s:\n%s\n", pFilename, err.c_str() );
		exit(-1);
	}
  readMesh.ClipEarTriangles();
  SetMesh(readMesh);
}

void MeshObject::SetMesh( rms::VFTriangleMesh & mesh )
{
	m_vOrigVerts.resize(0);
	m_vOrigNormals.resize(0);

	m_mesh.Clear(true);
	m_mesh.Copy(mesh);

	m_bvTree.SetMesh(&m_mesh);
	m_expmapgen.SetSurface(&m_mesh, &m_bvTree);
	m_bExpMapValid = false;

	if ( ! m_mesh.HasUVSet(0) )
		m_mesh.AppendUVSet();

	m_mesh.InitializeUVSet(0);

	m_mesh.GetEdgeLengthStats(m_fMinEdgeLength, m_fMaxEdgeLength, m_fAvgEdgeLength);
	m_fBoundaryWidth = m_fMaxEdgeLength * 1.0f;



	// set initial expmap
	Wml::Vector3f v,n;
	m_mesh.GetVertex(0, v, &n);
	return;
	m_vSeedFrame.Origin() = v;
	m_vSeedFrame.AlignZAxis( n );
	m_fDecalRadius = 10.0f * m_fAvgEdgeLength;
	m_bExpMapValid = false;

}

void MeshObject::NotifyMeshModified()
{
	m_bvTree.SetMesh(&m_mesh);
	m_expmapgen.SetSurface(&m_mesh, &m_bvTree);
	m_bExpMapValid = false;
}



bool MeshObject::FindIntersection( Wml::Ray3f & vRay, Wml::Vector3f & vHit, Wml::Vector3f & vHitNormal )
{
	rms::IMesh::TriangleID tID;
	if ( ! m_bvTree.FindRayIntersection( vRay.Origin, vRay.Direction, vHit, tID ) )
		return false;
	Wml::Vector3f vVerts[3], vNorms[3];
	m_mesh.GetTriangle(tID, vVerts, vNorms);

	float fBary[3];
	rms::BarycentricCoords( vVerts[0], vVerts[1], vVerts[2], vHit, fBary[0], fBary[1], fBary[2] );
	vHitNormal = 
		fBary[0]*vNorms[0] + fBary[1]*vNorms[1] + fBary[2]*vNorms[2];
	vHitNormal.Normalize();

	return true;
}

bool MeshObject::FindHitFrame( Wml::Ray3f & vRay, rms::Frame3f & vFrame )
{
	Wml::Vector3f vHit, vHitNormal;
	if ( ! FindIntersection(vRay, vHit, vHitNormal ) ) 
		return false;
	vFrame.Origin() = vHit;
	vFrame.AlignZAxis(vHitNormal);	
	return true;
}

bool MeshObject::FindNearestFrame( const Wml::Vector3f & vPoint, rms::Frame3f & vNearestFrame )
{
	rms::IMesh::TriangleID tID;
	if ( ! m_bvTree.FindNearest( vPoint, vNearestFrame.Origin(), tID ) )
		return false;
	Wml::Vector3f vVerts[3], vNorms[3];
	m_mesh.GetTriangle(tID, vVerts, vNorms);

	float fBary[3];
	rms::BarycentricCoords( vVerts[0], vVerts[1], vVerts[2], vNearestFrame.Origin(), fBary[0], fBary[1], fBary[2] );
	Wml::Vector3f vNormal =
		fBary[0]*vNorms[0] + fBary[1]*vNorms[1] + fBary[2]*vNorms[2];
	vNormal.Normalize();

	vNearestFrame.AlignZAxis( vNormal );
	return true;
}



void MeshObject::SetComputeExpMap(bool bEnable)
{
	m_bComputeExpMap = bEnable;
	m_bExpMapValid = false;
}

void MeshObject::MoveExpMap( rms::Frame3f & vFrame )
{
	m_vSeedFrame.Origin() = vFrame.Origin();
	m_vSeedFrame.AlignZAxis( vFrame.Z() );
	m_bExpMapValid = false;
}

void MeshObject::ScaleExpMap( float fScale )
{
	if ( fScale <= 0 )
		return;
	m_fDecalRadius *= fScale;
	if ( m_fDecalRadius < 0.01f )
		m_fDecalRadius = 0.01f;
	m_bExpMapValid = false;
}

void MeshObject::SetDecalRadius( float fRadius )
{
	m_fDecalRadius = fRadius;
	m_bExpMapValid = false;
}

void MeshObject::RotateExpMap( float fRotate )
{
	Wml::Matrix3f matRotate;
	matRotate.FromAxisAngle(m_vSeedFrame.Z(), fRotate);
	m_vSeedFrame.Rotate(matRotate);
	m_bExpMapValid = false;
}


//extern bool g_bUseClipPoly;
//extern rms::Polygon2f g_ClipPoly;


bool MeshObject::ValidateExpMap(bool bForce)
{
	if ( ! bForce ) {
		if ( ! m_bComputeExpMap )
			return false;
		if ( m_bExpMapValid )
			return false;
	}
	float fParamRadius = m_fDecalRadius + m_fBoundaryWidth;

	rms::Frame3f vNearestFrame(m_vSeedFrame);
	if ( ! FindNearestFrame( m_vSeedFrame.Origin(), vNearestFrame ) )
		// DebugBreak();
	if ( (m_vSeedFrame.Origin() - vNearestFrame.Origin()).Length() > 0.001f )
		m_vSeedFrame = vNearestFrame;

	//m_expmapgen.EnableClipPoly(g_bUseClipPoly);
	//if ( g_bUseClipPoly )
	//	m_expmapgen.SetClipPoly( g_ClipPoly );

	m_expmapgen.SetSurfaceDistances(m_vSeedFrame.Origin(), 0.0f, 
		fParamRadius, &m_vSeedFrame);
	m_expmapgen.CopyVertexUVs(&m_mesh, 0);
	m_bExpMapValid = true;
	return true;
}





class SplineProjecter {
public:
	SplineProjecter() { fX = fY = fZ = NULL; }
	SplineProjecter( rms::VFTriangleMesh & mesh, float fScale, float fSmooth  ) {
		Set(mesh,fScale,fSmooth);
	}
	~SplineProjecter() 
		{  ClearSplines(); }

	void Set( rms::VFTriangleMesh & mesh, float fScale, float fSmooth ) {
		ClearSplines();
		rms::VFTriangleMesh::vertex_iterator
			curv(mesh.BeginVertices()), endv(mesh.EndVertices());
		while ( curv != endv ) {
			rms::IMesh::VertexID vID = *curv;  ++curv;
			Wml::Vector2f vUV;
			if ( ! mesh.GetUV(vID, 0, vUV) )
				continue;
			vU.push_back(vUV.X() * fScale);
			vV.push_back(vUV.Y() * fScale);
			Wml::Vector3f v3D;
			mesh.GetVertex(vID, v3D);
			vX.push_back(v3D.X());
			vIDs.push_back(vID);
			vY.push_back(v3D.Y());
			vZ.push_back(v3D.Z());
		}
		MakeSplines(fSmooth);
	}

	Wml::Vector3f Project( const Wml::Vector2f & vUV ) 
		{ return Wml::Vector3f( (float)(*fX)(vUV.X(),vUV.Y()), (float)(*fY)(vUV.X(),vUV.Y()), (float)(*fZ)(vUV.X(),vUV.Y()) ); }
	Wml::Vector3f Project( double fU, double fV ) 
		{ return Wml::Vector3f( (float)(*fX)(fU, fV), (float)(*fY)(fU, fV), (float)(*fZ)(fU, fV) ); }

	const std::vector<rms::IMesh::VertexID> & VertIDs() { return vIDs; }
	Wml::Vector2f GetUV( int i ) { return Wml::Vector2f((float)vU[i], (float)vV[i]); }

protected:
	void MakeSplines(float fSmooth) {
		int nSize = (int)vU.size();
		fX = new Wml::IntpThinPlateSpline2d(nSize, &vU[0], &vV[0], &vX[0], fSmooth, false);
		fY = new Wml::IntpThinPlateSpline2d(nSize, &vU[0], &vV[0], &vY[0], fSmooth, false);
		fZ = new Wml::IntpThinPlateSpline2d(nSize, &vU[0], &vV[0], &vZ[0], fSmooth, false);
	}
	void ClearSplines() {
		if ( fX ) delete fX; fX = NULL;
		if ( fY ) delete fY; fY = NULL;
		if ( fZ ) delete fZ; fZ = NULL; 
		vIDs.resize(0); vU.resize(0); vV.resize(0); vX.resize(0);  vY.resize(0); vZ.resize(0);
	}
		

	std::vector<rms::IMesh::VertexID> vIDs;
	std::vector<double> vU, vV, vX, vY, vZ;
	Wml::IntpThinPlateSpline2d * fX, * fY, * fZ;
};



void MeshObject::ProjectPoints()
{
	float fScale = 1.0f / (m_fDecalRadius * (float)sqrt(2.0f));	
	SplineProjecter p(m_mesh, fScale, 0.001f);

	unsigned int nStep = 9;
	double fDelta = 1.0 / (double)(nStep-1);
	for ( unsigned int yi = 0; yi < nStep; ++yi ) {
		for ( unsigned int xi = 0; xi < nStep; ++xi ) {
			double fU = -0.5 + (double)xi * fDelta;
			double fV = -0.5 + (double)yi * fDelta;
			Wml::Vector3f v3D( p.Project(fU,fV) );
			m_vProjPoints.push_back(v3D);
		}
	}

}



struct VertMapInfo {
	rms::VFTriangleMesh mesh;
	std::vector<rms::IMesh::VertexID> vMapOldToNew;
	std::vector<rms::IMesh::VertexID> vMapNewToOld;
};

void MeshObject::RemeshDecal()
{
	// DebugBreak();
}




void MeshObject::DoRandomEdgeFlips(int nCount)
{
	// DebugBreak();
}



void MeshObject::AddNoise(float fConstant)
{
//	srand(time(NULL));
	srand(31337777);

	m_vOrigVerts.resize(m_mesh.GetVertexCount());
	rms::VFTriangleMesh::vertex_iterator curv(m_mesh.BeginVertices()), endv(m_mesh.EndVertices());
	while ( curv != endv ) {
		rms::IMesh::VertexID vID = *curv;  ++curv;
		Wml::Vector3f vVertex, vNormal;
		m_mesh.GetVertex(vID, vVertex, &vNormal);
		m_vOrigVerts[vID] = vVertex;
		
		// add noise
		float fNoise = (float)rand() / (float)RAND_MAX;
		fNoise -= 0.5f;
		fNoise *= 2.0f;
		fNoise *= fConstant;
		Wml::Vector3f vNoisyVertex = vVertex + vNormal * fNoise;

		m_mesh.SetVertex(vID, vNoisyVertex);

		float fDelta = (vVertex - vNoisyVertex).Length();
                _RMSInfo("Noise Delta %d: %f  %f\n", vID, fDelta, fNoise);
	}
}


struct VertData {
	std::vector<Wml::Vector3f> vInterpPos;
	std::vector<Wml::Vector2f> vUV;
	float fMaxDist;
};

void MeshObject::Smooth()
{
	std::vector<VertData> vVerts;
	vVerts.resize( m_mesh.GetVertexCount() );
	SplineProjecter p;

	float fMin, fMax, fAverage;
	m_mesh.GetEdgeLengthStats(fMin, fMax, fAverage);
	float fMaxEdgeLen = fMax;

	rms::VFTriangleMesh::vertex_iterator curv(m_mesh.BeginVertices()), endv(m_mesh.EndVertices());
	while ( curv != endv ) {
		rms::IMesh::VertexID vID = *curv;  ++curv;
		Wml::Vector3f vVertex, vNormal;
		m_mesh.GetVertex(vID, vVertex, &vNormal);
		rms::Frame3f vFrame(vVertex);
		vFrame.AlignZAxis(vNormal);

		m_expmapgen.SetSurfaceDistances(vFrame, fMaxEdgeLen, 30);
//		m_expmapgen.SetSurfaceDistances(vFrame.Origin(), fMaxEdgeLen, 2.5*fMaxEdgeLen, &vFrame);
		m_expmapgen.CopyVertexUVs(&m_mesh, 0);
		p.Set( m_mesh, 1.0f, 0.1f );

		const std::vector<rms::IMesh::VertexID> & vIDs = p.VertIDs();
		size_t nCount = vIDs.size();

		if ( vID % 100 == 0 )
                        _RMSInfo("Processing %d of %d  (%zu)\n", vID, m_mesh.GetVertexCount(), nCount);
			printf("Processing %d of %d  (%zu)\n", vID, m_mesh.GetVertexCount(), nCount);

                _RMSInfo("  %d verts\n", nCount);
		float fMaxDist = 0.0f;
		for ( unsigned int i = 0; i < nCount; ++i ) {
			Wml::Vector2f vUV = p.GetUV(i);
			vVerts[vIDs[i]].vUV.push_back(vUV);
			Wml::Vector3f v3D = p.Project(vUV);
			vVerts[vIDs[i]].vInterpPos.push_back(v3D);
			float fDist = vUV.Length();
			if ( fDist > fMaxDist )
				fMaxDist = fDist;
		}
	}

	// ok, now average and flatten
	float fErrSum = 0.0f;
	float fRealErrSum = 0.0f;
	curv = m_mesh.BeginVertices();
	while ( curv != endv ) {
		rms::IMesh::VertexID vID = *curv;  ++curv;
		Wml::Vector3f vVertex, vNormal;
		m_mesh.GetVertex(vID, vVertex, &vNormal);

		VertData & v = vVerts[vID];
		Wml::Vector3f vAverage(Wml::Vector3f::ZERO);
		float fWeightSum = 0.0f;
		size_t nCount = v.vInterpPos.size();
		for ( unsigned int i = 0; i < nCount; ++i ) {
			float fDist = v.vUV[i].Length();
			float fWeight = 1.0f / (0.1f + fDist);
			vAverage += fWeight * v.vInterpPos[i];
			fWeightSum += fWeight;
		}
		//vAverage *= (float)(1.0f / nCount);
		vAverage *= (float)(1.0f / fWeightSum);

		float fDelta = (vAverage - vVertex).Length();
		float fRealDelta = (m_vOrigVerts[vID] - vAverage).Length();
                _RMSInfo("Repair Delta %d: %f  %f\n", vID, fDelta, fRealDelta);
		fErrSum += fDelta;
		fRealErrSum += fRealDelta;

		m_mesh.SetVertex(vID, vAverage);
	}
        _RMSInfo("Final Deltas: %f %f\n", fErrSum, fRealErrSum);
  printf("Final Deltas: %f %f\n", fErrSum, fRealErrSum);

	m_bExpMapValid = false;

}


void MeshObject::SaveCurrent()
{
	m_vOrigVerts.resize( m_mesh.GetMaxVertexID() );
	m_vOrigNormals.resize( m_mesh.GetMaxVertexID() );
	rms::VFTriangleMesh::vertex_iterator curv(m_mesh.BeginVertices()), endv(m_mesh.EndVertices());
	while ( curv != endv ) {
		rms::IMesh::VertexID vID = *curv;  ++curv;
		Wml::Vector3f vCur, vNorm;
		m_mesh.GetVertex(vID, m_vOrigVerts[vID], &m_vOrigNormals[vID]);
	}	
}

void MeshObject::RestoreOriginal()
{
	if ( m_vOrigVerts.size() == 0 )
		return;

	rms::VFTriangleMesh::vertex_iterator curv(m_mesh.BeginVertices()), endv(m_mesh.EndVertices());
	while ( curv != endv ) {
		rms::IMesh::VertexID vID = *curv;  ++curv;
		Wml::Vector3f vCur, vNorm;
		m_mesh.GetVertex(vID, vCur, &vNorm);

		if ( m_vOrigNormals.size() > 0 ) {
			m_mesh.SetVertex( vID, m_vOrigVerts[vID], & m_vOrigNormals[vID] );
			m_vOrigVerts[vID] = vCur;
			m_vOrigNormals[vID] = vNorm;
		} else {
			m_mesh.SetVertex(vID, m_vOrigVerts[vID]);
			m_vOrigVerts[vID] = vCur;
		}
	}
}



void MeshObject::Render(bool bWireframe, bool bFlatShading, bool bUseScalarColors,
						rms::VFMeshRenderer::ColorTransferMode eTransferMode, int nScalarSet )
{
	glPushAttrib(GL_ENABLE_BIT | GL_POLYGON_BIT | GL_LINE_BIT | GL_POINT_BIT | GL_DEPTH_BUFFER_BIT );

	// render base mesh
	rms::VFMeshRenderer renderer(&m_mesh);
	renderer.SetNormalMode( (bFlatShading) ? rms::VFMeshRenderer::FaceNormals : rms::VFMeshRenderer::VertexNormals);
	renderer.SetEnableScalarColor( bUseScalarColors );
	renderer.SetColorTransferMode( eTransferMode );
	renderer.SetScalarColorSet( nScalarSet );
	renderer.SetDrawNormals(false);

//	glColor3f(0.6f, 1.0f, 0.6f);
//	glColor3f(1.0f, 1.0f, 1.0f);
	glColor3f(0.8f, 0.8f, 1.0f);
	if ( true ) {
		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(1.0f,1.0f);
	}
	renderer.Render();
	if ( bWireframe ) {
		glDisable(GL_POLYGON_OFFSET_FILL);
	}

	if ( bWireframe ) {
		glColor3f(0.0f, 0.0f, 0.0f);
		glPushAttrib(GL_ENABLE_BIT | GL_POLYGON_BIT | GL_LINE_BIT);
		glDisable(GL_LIGHTING);

		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(1.0f,1.0f);

		// render mesh as z-fill
		//glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
		//renderer.Render();
		//glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

		glDisable(GL_POLYGON_OFFSET_FILL);

		glLineWidth(3.0f);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

		bool bOriginal = renderer.GetEnableScalarColor();
		renderer.SetEnableScalarColor(false);
		renderer.Render();
		renderer.SetEnableScalarColor(bOriginal);

		glPopAttrib();
	}


	glDepthFunc(GL_LEQUAL);

	//// render support region of decal parameterization
	//renderer.SetTexture2DMode( rms::VFMeshRenderer::VertexTexture2D_Required );
	//glColor4f(0.5f, 0.5f, 1.0f, 1.0f);
	//renderer.Render();

	// do texture rendering
	if ( m_bExpMapValid ) {
		// enable transparency and alpha test
		glDepthFunc(GL_LEQUAL);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glEnable(GL_BLEND);
		glAlphaFunc( GL_NOTEQUAL, 0.0f );
		glEnable( GL_ALPHA_TEST );

		// enable texture
	//	m_texture.Enable();
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

		// set texture matrix transformation so that decal is in the right spot
		glMatrixMode(GL_TEXTURE);
		glPushMatrix();
		//glTranslatef( 0.5f, 0.5f, 0.0f );	
		//float fScale = 1.0f / (m_fDecalRadius * (float)sqrt(2.0f));
		//glScalef( fScale, fScale, 1.0f );
		glScalef( 1.5f, 1.5f, 1.5f );
		glMatrixMode(GL_MODELVIEW);

		renderer.SetTexture2DMode( rms::VFMeshRenderer::VertexTexture2D_Required );
		glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
		renderer.Render();

		glMatrixMode(GL_TEXTURE);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);

	//	m_texture.Disable();
	}


	// draw seed point frame
#if 0
	glDisable(GL_LIGHTING);

	glLineWidth(5.0f);
	glBegin(GL_LINES);
	glColor3f(1.0f,0.0f,0.0f);
	glVertex3fv(m_vSeedFrame.Origin());
	glVertex3fv(m_vSeedFrame.Origin() + 0.025f * m_vSeedFrame.X());
	glColor3f(0.0f,1.0f,0.0f);
	glVertex3fv(m_vSeedFrame.Origin());
	glVertex3fv(m_vSeedFrame.Origin() + 0.025f * m_vSeedFrame.Y());
	glColor3f(0.0f,0.0f,1.0f);
	glVertex3fv(m_vSeedFrame.Origin());
	glVertex3fv(m_vSeedFrame.Origin() + 0.025f * m_vSeedFrame.Z());
	glEnd();
#endif

	// draw projected points
	glPointSize(6.0f);
	glColor3f(0.0f, 0.0f, 0.0f);
	glBegin(GL_POINTS);
	for ( unsigned int i = 0; i < m_vProjPoints.size(); ++i )
		glVertex3fv(m_vProjPoints[i]);
	glEnd();

	glPopAttrib();
}



void MeshObject::RenderParamMesh()
{
	glPushAttrib(GL_ENABLE_BIT | GL_POLYGON_BIT | GL_LINE_BIT);
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
	glLineWidth(2.0f);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glColor3f(0.0f, 0.0f, 0.0f);

	// set transformation so that decal is in the right spot
	glPushMatrix();
	glLoadIdentity();
	//glTranslatef( 0.5f, 0.5f, 0.0f );	
	//float fScale = 1.0f / (m_fDecalRadius * (float)sqrt(2.0f));
	float fParamRadius = m_fDecalRadius + m_fBoundaryWidth;
	float fScale = 1.5f / (fParamRadius * (float)sqrt(2.0f));
	glScalef( fScale, fScale, 1.0f );

	rms::VFMeshRenderer renderer(&m_mesh);
	renderer.Render_UV();	

	glColor3f(1.0f, 0.0f, 0.0f);
	glLineWidth(3.0f);
	glBegin(GL_LINE_LOOP);
	float fSize = m_fDecalRadius / (float)sqrt(2.0f);
	glVertex2f(-fSize, -fSize);
	glVertex2f(fSize, -fSize);
	glVertex2f(fSize, fSize);
	glVertex2f(-fSize, fSize);
	glEnd();

	glPopMatrix();

	glPopAttrib();
}


struct BoundaryInfoEntry {
	BoundaryInfoEntry() {}
	BoundaryInfoEntry(rms::IMesh::VertexID id, Wml::Vector2f uv ) { vID = id; vUV = uv; fDist = vUV.Length(); }
	BoundaryInfoEntry(rms::IMesh::VertexID id, float dist ) { vID = id; vUV = Wml::Vector2f::ZERO; fDist = fDist; }
	rms::IMesh::VertexID vID;
	Wml::Vector2f vUV;
	float fDist;
	bool operator<( const BoundaryInfoEntry & b2 ) const {
		return fDist < b2.fDist;
	}
};

struct BoundaryInfo {
	std::vector<BoundaryInfoEntry> vVerts;
};


void MeshObject::ExportCSV_Edges(const char * pFilename)
{
	std::ofstream out(pFilename);
	//if (! out )
	//	DebugBreak();

	// make vector of boundary vertices
	std::vector<bool> bBoundary(m_mesh.GetVertexCount(), false);
	std::vector<unsigned int> vRewrite( m_mesh.GetVertexCount() );
	int nRewriteCounter = 0;
	rms::VFTriangleMesh::vertex_iterator curv(m_mesh.BeginVertices()), endv(m_mesh.EndVertices());
	while ( curv != endv ) {
		rms::IMesh::VertexID vID = *curv;  ++curv;
		bBoundary[vID] = m_mesh.IsBoundaryVertex(vID);
		if ( bBoundary[vID] )
			vRewrite[vID] = nRewriteCounter++;
		else
			vRewrite[vID] = rms::IMesh::InvalidID;
	}
	float fMin, fMax, fAverage;
	m_mesh.GetEdgeLengthStats(fMin, fMax, fAverage);
	float fMaxEdgeLen = fMax;

	size_t nCount = bBoundary.size();
	std::vector<BoundaryInfo> vBoundary;
	vBoundary.resize(nCount);

	unsigned int nMinNbrs = 99999;
	for ( unsigned int i = 0; i < nCount; ++i ) {
		if ( ! bBoundary[i] )
			continue;

		// find expmap and copy params
		Wml::Vector3f vVert, vNormal;
		m_mesh.GetVertex(i, vVert, &vNormal);
		rms::Frame3f vFrame(vVert);  vFrame.AlignZAxis(vNormal);
		m_expmapgen.SetSurfaceDistances( vFrame.Origin(), 0.0f, 3.0f * fMaxEdgeLen, &vFrame);
		m_expmapgen.CopyVertexUVs(&m_mesh, 0);
		
		// find all param'd neighbour boundary verts
		rms::IMesh::UVSet & uvset = m_mesh.GetUVSet(0);
		for ( unsigned int j = 0; j < nCount; ++j ) {
			Wml::Vector2f vUV;
			if ( i ==j || ! bBoundary[j]  ||  ! m_mesh.GetUV(j, 0, vUV) )
				continue;

			vBoundary[i].vVerts.push_back( BoundaryInfoEntry(j, vUV) );

			//Wml::Vector3f vVert2;
			//m_mesh.GetVertex(j, vVert2);
			//vBoundary[i].vVerts.push_back( BoundaryInfoEntry(j, (vVert2-vVert).Length()) );
		}
		size_t nNbrs = vBoundary[i].vVerts.size();
		if ( nNbrs < nMinNbrs )
			nMinNbrs = (unsigned int)nNbrs;
	}

	for ( unsigned int i = 0; i < nCount; ++i ) {
		if ( ! bBoundary[i] )
			continue;

		Wml::Vector3f vVert, vNormal;
		m_mesh.GetVertex(i, vVert, &vNormal);

		BoundaryInfo & info = vBoundary[i];
		std::sort( info.vVerts.begin(), info.vVerts.end() );

		out << vVert.X() << ", " << vVert.Y() << ", " << vVert.Z() << ", ";
		out << nMinNbrs << ", ";

		for ( unsigned int k = 0; k < nMinNbrs; ++k )
			out << vRewrite[info.vVerts[k].vID] << ", " << info.vVerts[k].vUV.X() << ", " << info.vVerts[k].vUV.Y() << ", ";
//			out << vRewrite[info.vVerts[k].vID] << ", " << info.vVerts[k].fDist << ", " << 0.0f << ", ";
		out << std::endl;
	}

	out.close();
}
