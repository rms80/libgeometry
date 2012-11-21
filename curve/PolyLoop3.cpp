// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#include "PolyLoop3.h"

#include <Wm4Segment3.h>
#include <Wm4DistVector3Segment3.h>
#include <VectorUtil.h>
#include <limits>
#include <fstream>
#include <opengl.h>

#include "rmsdebug.h"

using namespace rms;


unsigned int IPolyCurve::InvalidID = std::numeric_limits<unsigned int>::max();



template <class Real>
PolyLoop3<Real>::PolyLoop3()
{
}

template <class Real>
PolyLoop3<Real>::PolyLoop3( const PolyLoop3<Real> & copy )
{
	m_vVertices = copy.m_vVertices;
}

template <class Real>
PolyLoop3<Real>::~PolyLoop3()
{
}



template <class Real>
void PolyLoop3<Real>::FlipOrientation()
{
	size_t nCount = m_vVertices.size();
	for (unsigned int i = 0; i < nCount/2; ++i) {
		Wml::Vector3<Real> tmp = m_vVertices[i];
		m_vVertices[i] = m_vVertices[ (nCount-1)-i ];
		m_vVertices[ (nCount-1)-i ] = tmp;
	}	
}



template <class Real>
Real PolyLoop3<Real>::SqrDistance( const Wml::Vector3<Real> & vPoint, Wml::Vector3<Real> * pNearest)
{
	Real fMinSqrDist = std::numeric_limits<Real>::max();

	Wml::Vector3<Real> vMinNearest;
	unsigned int nMinIndex = std::numeric_limits<unsigned int>::max();

	size_t nCount = m_vVertices.size();
	for ( unsigned int i = 0; i < nCount-1; ++i ) {

		Wml::Segment3<Real> seg;
		seg.Origin = m_vVertices[i];
		seg.Direction = m_vVertices[ i+1 ] - seg.Origin;
		seg.Extent = seg.Direction.Normalize();

		Wml::DistVector3Segment3<Real> dist(vPoint, seg);
		Real fSqrDist = dist.GetSquared();
		if ( fSqrDist < fMinSqrDist ) {
			fMinSqrDist = fSqrDist;
			vMinNearest = dist.GetClosestPoint1();
			nMinIndex = i;
		}
	}

	if ( pNearest != NULL && nMinIndex != std::numeric_limits<unsigned int>::max() ) {
		*pNearest = vMinNearest;
	}

	return fMinSqrDist;
}






// bounding things
template <class Real>
void PolyLoop3<Real>::BoundingBox( Wml::AxisAlignedBox3<Real> & box ) const
{
	if( IsEmpty() ) {
		box = Wml::AxisAlignedBox3<Real>();
		return;
	}

	box = Wml::AxisAlignedBox3<Real>( m_vVertices[0].X(), m_vVertices[0].X(), m_vVertices[0].Y(), m_vVertices[0].Y(), m_vVertices[0].Z(), m_vVertices[0].Z() );
	size_t nVertices = m_vVertices.size();
	for (unsigned int i = 1; i < nVertices; ++i)
		rms::Union(box, m_vVertices[i]);
}


template < class Real>
Wml::Vector3<Real> PolyLoop3<Real>::Centroid() const
{
	Wml::Vector3<Real> vCentroid;
	size_t nVertices = m_vVertices.size();
	for (unsigned int i = 1; i < nVertices; ++i)
		vCentroid += m_vVertices[i];
	vCentroid /= (float)nVertices;
	return vCentroid;
}



// translation / rotation functions
template <class Real>
void PolyLoop3<Real>::Transform( const Wml::Matrix3<Real> & transform, const Wml::Vector3<Real> & vOrigin )
{
	size_t nCount = m_vVertices.size();
	for (unsigned int i = 0; i < nCount; ++i)
		m_vVertices[i] = transform * ( m_vVertices[i] - vOrigin ) + vOrigin;
}

template <class Real>
void PolyLoop3<Real>::Translate( Real fX, Real fY, Real fZ )
{
	size_t nCount = m_vVertices.size();
	for (unsigned int i = 0; i < nCount; ++i) {
		m_vVertices[i].X() += fX;
		m_vVertices[i].Y() += fY;
		m_vVertices[i].Z() += fZ;
	}
}

template <class Real>
void PolyLoop3<Real>::Scale( Real fScaleX, Real fScaleY, Real fScaleZ, const Wml::Vector3<Real> & vOrigin )
{
	size_t nCount = m_vVertices.size();
	for (unsigned int i = 0; i < nCount; ++i) {
		m_vVertices[i].X() = (m_vVertices[i].X() - vOrigin.X()) * fScaleX + vOrigin.X();
		m_vVertices[i].Y() = (m_vVertices[i].Y() - vOrigin.Y()) * fScaleY + vOrigin.Y();
		m_vVertices[i].Z() = (m_vVertices[i].Z() - vOrigin.Z()) * fScaleZ + vOrigin.Z();
	}
}




// Polygon simplification
// code adapted from: http://softsurfer.com/Archive/algorithm_0205/algorithm_0205.htm
// simplifyDP():
//  This is the Douglas-Peucker recursive simplification routine
//  It just marks vertices that are part of the simplified polyline
//  for approximating the polyline subchain v[j] to v[k].
//    Input:  tol = approximation tolerance
//            v[] = polyline array of vertex points
//            j,k = indices for the subchain v[j] to v[k]
//    Output: mk[] = array of markers matching vertex array v[]
template<class Real>
static void
simplifyDP( Real tol, std::vector<Wml::Vector3<Real> > & v, int j, int k, std::vector<bool> & mk )
{
    if (k <= j+1) // there is nothing to simplify
        return;

    // check for adequate approximation by segment S from v[j] to v[k]
    int     maxi = j;          // index of vertex farthest from S
    Real   maxd2 = 0;         // distance squared of farthest vertex
    Real   tol2 = tol * tol;  // tolerance squared
	Wml::Segment3<Real> S;    // segment from v[j] to v[k]
		S.Origin = v[j];
		S.Direction = v[k] - v[j];
		S.Extent = S.Direction.Normalize();
	Wml::Vector3<Real> u( S.Direction );  // segment direction vector
	Real cu = u.SquaredLength();            // segment length squared

    // test each vertex v[i] for max distance from S
    // compute using the Feb 2001 Algorithm's dist_Point_to_Segment()
    // Note: this works in any dimension (2D, 3D, ...)
	Wml::Vector3<Real> w;
	Wml::Vector3<Real> Pb;   // base of perpendicular from v[i] to S
	Real b, cw, dv2;         // dv2 = distance v[i] to S squared

    for (int i = j+1; i < k; i++)
    {
        // compute distance squared
		w = v[i] - S.Origin;
        cw = w.Dot(u);
        if ( cw <= 0 )
			dv2 = (v[i] - S.Origin).SquaredLength();
        else if ( cu <= cw )
			dv2 = (v[i] - (S.Origin + S.Direction)).SquaredLength();
        else {
            b = cw / cu;
            Pb = S.Origin + u * b;
			dv2 = (v[i] - Pb).SquaredLength();
        }
        // test with current max distance squared
        if (dv2 <= maxd2)
            continue;
        // v[i] is a new max vertex
        maxi = i;
        maxd2 = dv2;
    }
    if (maxd2 > tol2)        // error is worse than the tolerance
    {
        // split the polyline at the farthest vertex from S
        mk[maxi] = true;      // mark v[maxi] for the simplified polyline
        // recursively simplify the two subpolylines at v[maxi]
        simplifyDP( tol, v, j, maxi, mk );  // polyline v[j] to v[maxi]
        simplifyDP( tol, v, maxi, k, mk );  // polyline v[maxi] to v[k]
    }
    // else the approximation is OK, so ignore intermediate vertices
    return;
}



template <class Real>
void PolyLoop3<Real>::Simplify( bool bSimplifyStraightLines )
{
	Wml::AxisAlignedBox3<Real> box;
	BoundingBox(box);
	Real maxDim = std::max<Real>( box.Max[0] - box.Min[0], box.Max[1] - box.Min[1] );

	Real fTol = maxDim * (Real)0.025;		// this should be an argument...

	int n = (int)m_vVertices.size();

    int    i, k, pv;            // misc counters
    Real  tol2 = fTol * fTol;       // tolerance squared
	std::vector<Wml::Vector3<Real> > vt(n);  // vertex buffer
	std::vector<bool> mk(n, ! bSimplifyStraightLines);			 // marker buffer

    // STAGE 1.  Vertex Reduction within tolerance of prior vertex cluster
    vt[0] = m_vVertices[0];              // start at the beginning
    for (i = k = 1, pv = 0; i < n; i++) {
		if ( (m_vVertices[i] - m_vVertices[pv]).SquaredLength() < tol2 )
            continue;
        vt[k++] = m_vVertices[i];
        pv = i;
    }
    if (pv < n-1)
        vt[k++] = m_vVertices[n-1];      // finish at the end

    // STAGE 2.  Douglas-Peucker polyline simplification
	if (bSimplifyStraightLines) {
		mk[0] = mk[k-1] = 1;       // mark the first and last vertices
		simplifyDP( fTol * (Real)0.25, vt, 0, k-1, mk );
	}

	// copy marked vertices back to this polygon
	m_vVertices.resize(0);
	for (i = 0; i < k; ++i) {
		if (mk[i])
			m_vVertices.push_back( vt[i] );
	}

	return;
}











template <class Real>
void PolyLoop3<Real>::SmoothResample( float fFactor )
{
	Real fMaxEdgeLen = (Real)0.0;
	size_t nVerts = m_vVertices.size();
	for ( unsigned int i = 0; i  < nVerts; ++i ) {
		Real fLen = ( m_vVertices[(i+1) % nVerts] - m_vVertices[i] ).Length();
		if ( fLen > fMaxEdgeLen )
			fMaxEdgeLen = fLen;
	}
	Real fSnapDist = fMaxEdgeLen * (Real)0.5;

	std::vector<Wml::Vector3<Real> > vNewPts;
	bool bDone = false;
	while ( ! bDone ) {
		bDone = true;
		vNewPts.resize(0);

		vNewPts.push_back( m_vVertices[0] );

		Real fDistSum = 0.0f;
		for ( unsigned int i = 1; i < nVerts; ++i ) {
			fDistSum += (m_vVertices[i] - m_vVertices[i-1]).Length();
			if ( fDistSum > fSnapDist ) {
				vNewPts.push_back( m_vVertices[i] );
				fDistSum = 0.0f;
			} else
				bDone = false;
		}
		m_vVertices = vNewPts;
		nVerts = m_vVertices.size();
	}

	// ok now subdivide once
	nVerts = m_vVertices.size();
	vNewPts.resize(0);
	for ( unsigned int i = 1; i < nVerts+1; ++i ) {
		Wml::Vector3<Real> & vPrev = m_vVertices[i-1];
		Wml::Vector3<Real> & vCur = m_vVertices[ i % nVerts];
		Wml::Vector3<Real> & vNext = m_vVertices[(i+1) % nVerts];
		vNewPts.push_back(	(Real)0.25*vPrev + (Real)0.75*vCur );
		vNewPts.push_back(	(Real)0.75*vCur + (Real)0.25*vNext );
	}
	m_vVertices = vNewPts;
	
}




template <class Real>
void PolyLoop3<Real>::Render( bool bLines, bool bPoints )
{
	size_t nVerts = m_vVertices.size();

	if ( bLines ) {
		glBegin(GL_LINE_LOOP);
		for ( unsigned int k = 0; k < nVerts; ++k )
			glVertex3f( (float)m_vVertices[k][0], (float)m_vVertices[k][1], (float)m_vVertices[k][2] );
		glEnd();
	}
	if ( bPoints ) {
		glBegin(GL_POINTS);
		for ( unsigned int k = 0; k < nVerts; ++k )
			glVertex3f( (float)m_vVertices[k][0], (float)m_vVertices[k][1], (float)m_vVertices[k][2] );
		glEnd();
	}
}





//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
namespace rms
{
template class PolyLoop3<float>;
template class PolyLoop3<double>;
}
//----------------------------------------------------------------------------
