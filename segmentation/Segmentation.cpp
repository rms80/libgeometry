// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#include "Segmentation.h"

#include <VectorUtil.h>
#include <limits>
#include <opengl.h>
#include <rmsdebug.h>

using namespace rms;

unsigned int Segmentation::InvalidID = std::numeric_limits<unsigned int>::max();

std::vector<IMesh::TriangleID> Segmentation::EMPTY_SET;
std::vector<IMesh::VertexID> Segmentation::EMPTY_BOUNDARY;


Segmentation::Segmentation(VFTriangleMesh * pMesh)
{
	m_nIDCounter = 0;
	m_pMesh = pMesh;
}


void Segmentation::SetMesh(VFTriangleMesh * pMesh)
{
	m_nIDCounter = 0;
	m_pMesh = pMesh;
}

void Segmentation::Clear()
{
	m_SegmentMap.clear();
	m_vSegments.clear();
	m_nIDCounter = 0;
}


void hsl_to_rgb(float h, float s, float l,  float & r, float & g, float & b)
{
	float q = (l < 0.5) ? l*(1+s) : l+s-(l*s);
	float p = 2*l - q;
	float hk = (h/360.0f);
	float t[3], rgb[3];
	t[0] = fmod( hk + 1.0f/3.0f, 1);
	t[1] = fmod( hk, 1);
	t[2] = fmod( hk - 1.0f/3.0f, 1);
	for ( int i = 0; i < 3; ++i ) {
		if ( t[i] < (1.0f/6.0f) )
			rgb[i] = p + ((q-p)*6*t[i]);
		else if ( t[i] < 0.5f )
			rgb[i] = q;
		else if ( t[i] < 2.0f/3.0f )
			rgb[i] = p + ((q-p)*6*(2.0f/3.0f - t[i]));
		else
			rgb[i] = p;
	}

	r = rgb[0];
	g = rgb[1];
	b = rgb[2];
}

Segmentation::SegmentID Segmentation::AppendSegment(unsigned int nSizeHint, unsigned int nSetID)
{
	Segment vTris;
	vTris.id = ++m_nIDCounter;
	if ( nSetID != -1 )
		vTris.id = nSetID;

	float h = (float)( (72+1327*vTris.id) % 251 );
	float l = (float)( (18+337*vTris.id) % 37 ) / 37.0f;
	l = (l*0.5f) + 0.3f;
	//_RMSInfo("h %15.8f   l %16.8f\n", h, l);
	hsl_to_rgb(h,1,l, vTris.vColor[0], vTris.vColor[1], vTris.vColor[2]);

	m_vSegments.insert(vTris);
	return vTris.id;
}


void Segmentation::CreateSegments( const std::map<IMesh::TriangleID, int> & vSegments )
{
	std::map<IMesh::TriangleID, int>::const_iterator curf(vSegments.begin()), endf(vSegments.end());
	while ( curf != endf ) {
		IMesh::TriangleID tID = curf->first;
		SegmentID sID = curf->second;
		++curf;
		if (! m_pMesh->IsTriangle(tID) )
			continue;

		Segment * pSeg = FindSegment(sID);
		if ( pSeg == NULL )
			AppendSegment(0, sID);
		pSeg = FindSegment(sID);
		if ( ! pSeg )
			lgBreakToDebugger();
		pSeg->vTris.push_back(tID);
		m_SegmentMap[tID] = pSeg->id;
	}
}




Segmentation::Segment * Segmentation::FindSegment(SegmentID sID)
{
	Segment tmp; tmp.id = sID;
	std::set<Segment>::iterator found = m_vSegments.find(tmp);
	if ( found == m_vSegments.end() )
		return NULL;
	return &(*found);
}

Segmentation::SegmentID Segmentation::FindSet( IMesh::TriangleID tID )
{
	std::map<IMesh::TriangleID, SegmentID>::iterator found = m_SegmentMap.find(tID);
	if ( found != m_SegmentMap.end() )
		return (*found).second;
	else
		return InvalidID;
}


const std::vector<IMesh::TriangleID> & Segmentation::GetSet( SegmentID sID ) const
{
	// set has to exist
	Segment tmp; tmp.id = sID;
	std::set<Segment>::const_iterator found = m_vSegments.find(tmp);
	if (found == m_vSegments.end() )
		return EMPTY_SET;

	return (*found).vTris;
}

const std::vector<IMesh::VertexID> & Segmentation::GetBoundary( SegmentID sID ) const
{
	// set has to exist
	Segment tmp; tmp.id = sID;
	std::set<Segment>::const_iterator found = m_vSegments.find(tmp);
	if (found == m_vSegments.end() )
		return EMPTY_BOUNDARY;

	return (*found).vBoundary;
}





void Segmentation::SanityCheck(VFTriangleMesh & mesh)
{
	std::set<Segment>::iterator curt(m_vSegments.begin()), endt(m_vSegments.end());
	while ( curt != endt ) {
		Segment & t = *curt++;

		for ( unsigned int k = 0; k < t.vTris.size(); ++k )
			if ( ! mesh.IsTriangle( t.vTris[k] ) )
				lgBreakToDebugger();

		for ( unsigned int k = 0; k < t.vBoundary.size(); ++k )
			if ( ! mesh.IsVertex( t.vBoundary[k] ) )
				lgBreakToDebugger();
	}
}




void Segmentation::DebugRender()
{
	std::set<Segment>::const_iterator cur(m_vSegments.begin()), end(m_vSegments.end());
	while ( cur != end ) {
		const Segment & s = *cur++;
		
		glColor3fv(s.vColor);

		glBegin(GL_TRIANGLES);
		std::vector<IMesh::TriangleID>::const_iterator curt(s.vTris.begin()), endt(s.vTris.end());
		while ( curt != endt ) {
			IMesh::TriangleID tID = *curt++;
			Wml::Vector3f vTri[3];
			m_pMesh->GetTriangle(tID, vTri);
			glNormal3fv( rms::Normal(vTri[0], vTri[1], vTri[2]) );
			for ( int k = 0; k < 3; ++k )
				glVertex3fv(vTri[k]);
		}

		glEnd();
	}
}