// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#include "opengl.h"
#include "MeshGeodesic.h"
#include <limits>
#include <Wm4DistVector3Segment3.h>

#include "rmsdebug.h"


MeshGeodesic::MeshGeodesic( )
{
	m_pMesh = NULL;
	m_pBVTree = NULL;
	m_pExpGen = NULL;
	m_bIsValid = false;
	m_vPoint1.Origin().X() = std::numeric_limits<float>::max();
	m_vPoint2.Origin().X() = std::numeric_limits<float>::max();
}

MeshGeodesic::MeshGeodesic( const MeshGeodesic & m2 )
{
	m_pMesh = m2.m_pMesh;
	m_pBVTree = m2.m_pBVTree;
	m_pExpGen = m2.m_pExpGen;
	m_bIsValid = m2.m_bIsValid;
	m_vPoint1 = m2.m_vPoint1;
	m_vPoint2 = m2.m_vPoint2;
	m_vDijkstraPath = m2.m_vDijkstraPath;
	m_vOptPath = m2.m_vOptPath;
}

MeshGeodesic::~MeshGeodesic(void)
{
}

void MeshGeodesic::SetSurface(rms::VFTriangleMesh * pMesh, rms::IMeshBVTree * pBVTree, rms::ExpMapGenerator * pExpgen)
{
	m_pMesh = pMesh;
	m_pBVTree = pBVTree;
	m_pExpGen = pExpgen;

	m_bIsValid = false;

	m_vPoint1.Origin().X() = std::numeric_limits<float>::max();
	m_vPoint2.Origin().X() = std::numeric_limits<float>::max();
}


void MeshGeodesic::SetSurfacePoints( const std::vector<rms::Frame3f> & vCurve )
{
	if ( vCurve.size() < 3 )
		return;

	m_vDijkstraPath.clear();
	size_t nCount = vCurve.size();
	for ( unsigned int i = 0; i < nCount; ++i ) {
		rms::Frame3f vFrame;
		if ( i > 0 )
			vFrame = m_vDijkstraPath.back();
		//float fDist = (vFrame.Origin() - vCurve[i].Origin()).Length();
		//if ( fDist < 0.01f )
		//	continue;
		vFrame.Origin() = vCurve[i].Origin();
		vFrame.AlignZAxis( vCurve[i].Z() );
		m_vDijkstraPath.push_back(vFrame);
	}
	m_vOptPath = m_vDijkstraPath;	

	m_vPoint1 = m_vDijkstraPath.front();
	m_vPoint2 = m_vDijkstraPath.back();

	m_bIsValid = true;
}

void MeshGeodesic::Clear()
{
	m_vPoint1.Origin().X() = std::numeric_limits<float>::max();
	m_vPoint2.Origin().X() = std::numeric_limits<float>::max();
	m_bIsValid = false;
	m_vDijkstraPath.resize(0);
	m_vOptPath.resize(0);
	m_vLocalUVs.resize(0);
}


void MeshGeodesic::Validate()
{
	if ( m_vPoint1.Origin().X() == std::numeric_limits<float>::max() ||
		 m_vPoint2.Origin().X() == std::numeric_limits<float>::max() )
		 return;
	if ( m_bIsValid )
		return;

	// WRITE GENERIC DIJKSTRA ALGO!
	lgBreakToDebugger();

	//m_pExpGen->FindDijkstraPath( m_vPoint1, m_vPoint2 );
	//m_vDijkstraPath = m_pExpGen->GetDijkstraPath();
	//m_vOptPath = m_vDijkstraPath;	
	//m_vLocalUVs.resize(m_vOptPath.size());

	m_bIsValid = true;
}


void MeshGeodesic::OptimizePath()
{
	float fDijkstraLength = 0.0f;
	float fLengthBefore = 0.0f;
	size_t nCount1 = m_vDijkstraPath.size();
	if ( nCount1 == 0 )
		return;
	for ( unsigned int i = 0 ; i < nCount1-1; ++i ) 
		fDijkstraLength += (m_vDijkstraPath[i+1].Origin() - m_vDijkstraPath[i].Origin()).Length();
	size_t nCount2 = m_vOptPath.size();
	for ( unsigned int i = 0 ; i < nCount2-1; ++i ) 
		fLengthBefore += (m_vOptPath[i+1].Origin() - m_vOptPath[i].Origin()).Length();


	float fDist, fDist2;
	size_t nCount = m_vOptPath.size();
	for ( unsigned int i = 0; i < nCount-2; ++i ) {
		rms::Frame3f & vCenter = m_vOptPath[i];
		rms::Frame3f & vNext = m_vOptPath[i+1];
		rms::Frame3f & vNext2 = m_vOptPath[i+2];
		float fPathDist = 
			(vNext.Origin() - vCenter.Origin()).Length() +
			(vNext2.Origin() - vNext.Origin()).Length();
		if ( ! _finite(fPathDist) )
			lgBreakToDebugger();

		if ( true || fDijkstraLength - fLengthBefore == 0 ) 
			m_pExpGen->SetSurfaceDistances(vCenter.Origin(), 0.0f, fPathDist * 1.5f, &vCenter );
		else {
			//float fDotThresh = 0.5f;
			//m_pExpGen->ComputeExpMap(vCenter, fPathDist*1.5f, m_vLocalUVs[i], fDotThresh);
		}

		// generate mesh caches
		m_pExpGen->MeshCurrentUVs();

		Wml::Vector2f vUVNext = m_pExpGen->FindUV( vNext.Origin() );
		Wml::Vector2f vUVNext2 = m_pExpGen->FindUV( vNext2.Origin() );
		
		fDist = vUVNext.Length();		
		if ( ! _finite(fDist) ) lgBreakToDebugger();
		vUVNext.Normalize();
		fDist2 = vUVNext2.Length();	
		if ( ! _finite(fDist2) ) lgBreakToDebugger();
		vUVNext2.Normalize();
		Wml::Vector2f vNewUV = vUVNext2 * fDist;

		// save uv
		//m_vLocalUVs[i] = vNewUV;
		//m_vLocalUVs[i].Normalize();

		Wml::Vector3f vNewNorm;
		Wml::Vector3f vNew3D = m_pExpGen->Find3D( vNewUV, &vNewNorm );

		vNext.Origin() = vNew3D;
		vNext.AlignZAxis( vNewNorm );
	}

	// clean point set
	float fCleanerThreshold = 0.02f;
	std::vector<rms::Frame3f> vFrames;
	vFrames.push_back(m_vOptPath.front());
	float fAccumDist = 0.0f;
	for ( unsigned int i = 1; i < nCount; ++i ) {
		rms::Frame3f & vCenter = m_vOptPath[i];
		rms::Frame3f & vPrev = vFrames.back();
		float fDist = (vPrev.Origin() - vCenter.Origin()).Length();
		fAccumDist += fDist;
		if ( fAccumDist < fCleanerThreshold )
			continue;
		fAccumDist = 0.0f;
		vFrames.push_back(vCenter);
	}
	if ( fAccumDist > 0 )
		vFrames.push_back(m_vOptPath.back());
	m_vOptPath = vFrames;

	float fLengthAfter = 0.0f;
	for ( unsigned int i = 0 ; i < nCount2-1; ++i ) 
		fLengthAfter += (m_vOptPath[i+1].Origin() - m_vOptPath[i].Origin()).Length();
	_RMSInfo("DJ: %.5f  BEF: %.5f  AFT: %.5f  DT: %.5f\n",
		fDijkstraLength, fLengthBefore, fLengthAfter, (fLengthAfter-fLengthBefore) );
}



float MeshGeodesic::Distance( const Wml::Vector3f & vVertex, Wml::Vector3f & vNearest )
{
	float fMinDist = std::numeric_limits<float>::max();
	vNearest = Wml::Vector3f::ZERO;

	size_t nCount = m_vOptPath.size();
	for ( unsigned int i = 0; i < nCount-1; ++i ) {
		Wml::Segment3f seg;
		seg.Origin = m_vOptPath[i].Origin();
		seg.Direction = m_vOptPath[i+1].Origin() - seg.Origin; 
		seg.Extent = seg.Direction.Length();
		seg.Direction.Normalize();
		Wml::DistVector3Segment3f d( vVertex, seg );
		float fDist = d.Get();
		if ( fDist < fMinDist ) {
			fMinDist = fDist;
			vNearest = seg.Origin + d.GetSegmentParameter() * seg.Direction;
		}
	}
	return fMinDist;
}


void MeshGeodesic::Transport( const Wml::Vector2f & vDirectionIn, const rms::Frame3f * pPrevFrame )
{
	Wml::Vector2f vDirection(vDirectionIn);

	// align frames
	rms::Frame3f vStartFrame = m_vOptPath[0];
	if ( pPrevFrame ) {
		vStartFrame = *pPrevFrame;
		vStartFrame.Origin() = m_vOptPath[0].Origin();
		vStartFrame.AlignZAxis(m_vOptPath[0].Z());
	}

	size_t nCount = m_vOptPath.size();
	for ( unsigned int i = 0; i < nCount; ++i ) {
		rms::Frame3f vCurFrame( m_vOptPath[i] );
 		rms::Frame3f vNewFrame(vStartFrame);
		vNewFrame.AlignZAxis(vCurFrame.Z());
		vNewFrame.Origin() = vCurFrame.Origin();
		m_vOptPath[i] = vNewFrame;
	}

	float fLength = vDirection.Length() * 1.5f;
	for ( unsigned int i = 0; i < nCount; ++i ) {
		rms::Frame3f & vCenter = m_vOptPath[i];
		m_pExpGen->SetSurfaceDistances(vCenter.Origin(), 0.0f, fLength, &vCenter );

		// generate mesh caches
		m_pExpGen->MeshCurrentUVs();

		Wml::Vector3f vNew3D, vNewNormal;
		vNew3D = m_pExpGen->Find3D( vDirection, &vNewNormal );

		vCenter.Origin() = vNew3D;
		vCenter.AlignZAxis( vNewNormal );
	}

}

void MeshGeodesic::GetEndpoints( rms::Frame3f & vEndPoint1, rms::Frame3f & vEndPoint2 )
{
	vEndPoint1 = m_vOptPath.front();
	vEndPoint2 = m_vOptPath.back();
}

const std::vector<rms::Frame3f> & MeshGeodesic::GetPath()
{
	return m_vOptPath;
}


void MeshGeodesic::Render(const Wml::ColorRGBA & cEdgeColor, bool bDrawDijkstraPath, bool bDrawEndpoints)
{
	Validate();

	glPushAttrib(GL_ENABLE_BIT | GL_LINE_BIT | GL_POINT_BIT);
	glDisable(GL_LIGHTING);
	glDepthFunc(GL_LEQUAL);
	
	static const float fNormOff = 0.005f;
	size_t nCount = 0;

	glPointSize(10.0f);
	if ( bDrawEndpoints ) {
		glColor3f(1.0f, 0.5f, 0.0f);
		glBegin(GL_POINTS);
		glVertex3fv( m_vPoint1.Origin() + m_vPoint1.Z() * fNormOff );
		glVertex3fv( m_vPoint2.Origin() + m_vPoint2.Z() * fNormOff );
		glEnd();
	}

	glLineWidth(5.0f);

	if ( bDrawDijkstraPath ) {
		glColor3f(0.0f, 0.0f, 1.0f);
		glBegin(GL_LINE_STRIP);
		nCount = m_vDijkstraPath.size();
		for ( unsigned int i = 0; i < nCount; ++i ) {
			rms::Frame3f & frame = m_vDijkstraPath[i];
			Wml::Vector3f vNormOff = frame.Z() * fNormOff;
			glVertex3fv( frame.Origin() + vNormOff );
		}
		glEnd();
	}

	glColor4fv(cEdgeColor);
	glBegin(GL_LINE_STRIP);
	nCount = m_vOptPath.size();
	for ( unsigned int i = 0; i < nCount; ++i ) {
		rms::Frame3f & frame = m_vOptPath[i];
		Wml::Vector3f vNormOff = frame.Z() * fNormOff * 1.0f;
		glVertex3fv( frame.Origin() + vNormOff );
	}
	glEnd();

	glPointSize(4.0f);
	glColor3f(0.0f, 1.0f, 0.0f);
	glBegin(GL_POINTS);
	for ( unsigned int i = 0; i < nCount; ++i ) {
		rms::Frame3f & frame = m_vOptPath[i];
		Wml::Vector3f vNormOff = frame.Z() * fNormOff * 2.5f;
		glVertex3fv( frame.Origin() + vNormOff );
	}
	glEnd();



	glPopAttrib();
}