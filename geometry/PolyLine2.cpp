// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#include "PolyLine2.h"

#include <Wm4Segment2.h>
#include <Wm4DistVector2Segment2.h>
#include <VectorUtil.h>
#include <limits>
#include <fstream>

using namespace rms;


template <class Real>
PolyLine2<Real>::PolyLine2()
{
	m_fCacheLength = std::numeric_limits<Real>::max();
}

template <class Real>
PolyLine2<Real>::PolyLine2( const PolyLine2<Real> & copy )
{
	m_vVertices = copy.m_vVertices;
	m_fCacheLength = std::numeric_limits<Real>::max();
}

template <class Real>
PolyLine2<Real>::~PolyLine2()
{
}


template <class Real>
void PolyLine2<Real>::Reverse()
{
	size_t nCount = m_vVertices.size();
	for (unsigned int i = 0; i < nCount/2; ++i) {
		Wml::Vector2<Real> tmp = m_vVertices[i];
		m_vVertices[i] = m_vVertices[ (nCount-1)-i ];
		m_vVertices[ (nCount-1)-i ] = tmp;
	}	
}


// bounding things
template <class Real>
void PolyLine2<Real>::BoundingBox( Wml::AxisAlignedBox2<Real> & box ) const
{
	if( IsEmpty() ) {
		box = Wml::AxisAlignedBox2<Real>();
		return;
	}

	box = Wml::AxisAlignedBox2<Real>( m_vVertices[0].X(), m_vVertices[0].X(), m_vVertices[0].Y(), m_vVertices[0].Y() );
	size_t nVertices = m_vVertices.size();
	for (unsigned int i = 1; i < nVertices; ++i) {
		const Wml::Vector2<Real> & vtx = m_vVertices[i];
		if ( vtx.X() < box.Min[0] ) 
			box.Min[0] = vtx.X();
		else if (vtx.X() > box.Max[0] )
			box.Max[0] = vtx.X();
		if ( vtx.Y() < box.Min[1] ) 
			box.Min[1] = vtx.Y();
		else if (vtx.Y() > box.Max[1] )
			box.Max[1] = vtx.Y();
	}
}


template <class Real>
Real PolyLine2<Real>::GetLength() const
{
	size_t nVertices = m_vVertices.size();
	Real fSum = 0.0f;
	for ( unsigned int i = 0 ; i < nVertices-1; ++i )
		fSum += ( m_vVertices[i+1] - m_vVertices[i] ).Length();
	return fSum;
}


// translation / rotation functions
template <class Real>
void PolyLine2<Real>::Transform( const Wml::Matrix2<Real> & transform )
{
	size_t nCount = m_vVertices.size();
	for (unsigned int i = 0; i < nCount; ++i)
		m_vVertices[i] = transform * m_vVertices[i];
}

template <class Real>
void PolyLine2<Real>::Translate( Real fX, Real fY )
{
	size_t nCount = m_vVertices.size();
	for (unsigned int i = 0; i < nCount; ++i) {
		m_vVertices[i].X() += fX;
		m_vVertices[i].Y() += fY;
	}
}

template <class Real>
void PolyLine2<Real>::Scale( Real fScaleX, Real fScaleY, const Wml::Vector2<Real> & vOrigin )
{
	size_t nCount = m_vVertices.size();
	for (unsigned int i = 0; i < nCount; ++i) {
		m_vVertices[i].X() = (m_vVertices[i].X() - vOrigin.X()) * fScaleX + vOrigin.X();
		m_vVertices[i].Y() = (m_vVertices[i].Y() - vOrigin.Y()) * fScaleY + vOrigin.Y();
	}
}




template <class Real>
void PolyLine2<Real>::PrecomputeArcLengthParams()
{
	if ( m_fCacheLength != std::numeric_limits<Real>::max() )
		return;
	m_fCacheLength = GetLength();
	Real fLenScale = (Real)1.0 / m_fCacheLength;

	size_t nCount = m_vVertices.size();
	m_vVtxParams.resize( nCount );
	m_vVtxParams[0] = 0.0f;
	m_vVtxParams[nCount-1] = 1.0f;

	unsigned int nBuckets = 20;
	m_vParamTable.resize( nBuckets+1 );
	m_fCacheBucketStep = 1.0f / (nBuckets);
	m_vParamTable[0] = 0;
	unsigned int nCurBucket = 0;

	Real fAccumDist = 0.0f;
	for ( unsigned int i = 1; i < nCount-1; ++i ) {
		Real fDistDelta = ( m_vVertices[i] - m_vVertices[i-1] ).Length();
		fAccumDist += fDistDelta;
		Real fParam = fAccumDist / m_fCacheLength;
		m_vVtxParams[i] = fParam;

		// figure out which bucket this vertex goes into
		unsigned int nBucket = (unsigned int)(fParam / m_fCacheBucketStep);

		// if it's not nCurBucket, repeat nCurBucket up to nBucket, and then fill nBucket
		if ( nBucket > nCurBucket ) {
			++nCurBucket;
			while ( nCurBucket < nBucket ) {
				m_vParamTable[ nCurBucket ] = m_vParamTable[ nCurBucket-1 ];
				++nCurBucket;
			}
			m_vParamTable[ nCurBucket ] = i;
		}
	}

	// last bucket is always last vertex
	unsigned int nLastBucket = (unsigned int)m_vParamTable.size()-1;
	++nCurBucket;
	while ( nCurBucket < nLastBucket ) {
		m_vParamTable[ nCurBucket ] = m_vParamTable[ nCurBucket-1 ];
		++nCurBucket;
	}
	m_vParamTable[nLastBucket] = (unsigned int)nCount-1;
}

template <class Real>
Wml::Vector2<Real> PolyLine2<Real>::Vertex( Real fArcLengthParam )
{
	PrecomputeArcLengthParams();

	if ( fArcLengthParam < 0 )
		fArcLengthParam = 0;
	if ( fArcLengthParam > 1 )
		fArcLengthParam = 1;

	// find bucket to start at
	unsigned int nBucket = (unsigned int)(fArcLengthParam / m_fCacheBucketStep);
	unsigned int nVtx = m_vParamTable[nBucket];
	while ( m_vVtxParams[nVtx] < fArcLengthParam )
		++nVtx;

	unsigned int n1, n2;
	if ( nVtx == 0 ) {
		n1 = 0; n2 = 1;
	} else {
		n1 = nVtx-1;		// we overshot in the loop above
		n2 = nVtx;
	}
	Real fLow = m_vVtxParams[ n1 ];
	Real fHigh = m_vVtxParams[ n2 ];
//	fArcLengthParam -= fLow;
	Real fAlpha = 0.0f;
	if ( (Real)abs(fHigh-fLow) > (Real)0.00001 )
		fAlpha = ( fArcLengthParam - fLow ) / ( fHigh - fLow );
	return ((Real)1.0-fAlpha)*m_vVertices[n1] + fAlpha*m_vVertices[n2];
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
simplifyDP( Real tol, std::vector<Wml::Vector2<Real> > & v, int j, int k, std::vector<bool> & mk )
{
    if (k <= j+1) // there is nothing to simplify
        return;

    // check for adequate approximation by segment S from v[j] to v[k]
    int     maxi = j;          // index of vertex farthest from S
    Real   maxd2 = 0;         // distance squared of farthest vertex
    Real   tol2 = tol * tol;  // tolerance squared
	Wml::Segment2<Real> S;    // segment from v[j] to v[k]
		S.Origin = v[j];
		S.Direction = v[k] - v[j];
	Wml::Vector2<Real> u( S.Direction );  // segment direction vector
	Real cu = u.SquaredLength();            // segment length squared

    // test each vertex v[i] for max distance from S
    // compute using the Feb 2001 Algorithm's dist_Point_to_Segment()
    // Note: this works in any dimension (2D, 3D, ...)
	Wml::Vector2<Real> w;
	Wml::Vector2<Real> Pb;   // base of perpendicular from v[i] to S
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
void PolyLine2<Real>::Simplify( bool bSimplifyStraightLines )
{
	Wml::AxisAlignedBox2<Real> box;
	BoundingBox(box);
	Real maxDim = std::max<Real>( box.Max[0] - box.Min[0], box.Max[1] - box.Min[1] );

	Real fTol = maxDim * (Real)0.025;		// this should be an argument...

	int n = (int)m_vVertices.size();

    int    i, k, pv;            // misc counters
    Real  tol2 = fTol * fTol;       // tolerance squared
	std::vector<Wml::Vector2<Real> > vt(n);  // vertex buffer
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
Real PolyLine2<Real>::SqrDistance( const Wml::Vector2<Real> & vPoint, Wml::Vector2<Real> * pNearest)
{
	Real fMinSqrDist = std::numeric_limits<Real>::max();

	Wml::Vector2<Real> vMinNearest;
	unsigned int nMinIndex = std::numeric_limits<unsigned int>::max();

	size_t nCount = m_vVertices.size();
	for ( unsigned int i = 0; i < nCount-1; ++i ) {

		Wml::Segment2<Real> seg;
		seg.Origin = m_vVertices[i];
		seg.Direction = m_vVertices[ i+1 ] - seg.Origin;
		seg.Extent = seg.Direction.Normalize();

		Wml::DistVector2Segment2<Real> dist(vPoint, seg);
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


template <class Real>
bool PolyLine2<Real>::Load( const char * pFilename )
{
	std::ifstream in(pFilename);
	if (!in)
		return false;

	unsigned int nCount;
	in >> nCount;

	m_vVertices.resize(nCount);

	for ( unsigned int i = 0; i < nCount; ++i ) {

		Real fX, fY;
		in >> fX >> fY;
		if (!in) {
			in.close();
			return false;
		}

		m_vVertices[i] = Wml::Vector2<Real>(fX,fY);
	}

	in.close();
	return true;
}

template <class Real>
bool PolyLine2<Real>::Save( const char * pFilename ) const
{
	std::ofstream out(pFilename);
	if (!out)
		return false;

	size_t nCount = m_vVertices.size();
	out << (unsigned int)nCount << std::endl;

	for ( unsigned int i = 0; i < nCount; ++i ) {
		out << m_vVertices[i].X() << " " << m_vVertices[i].Y() << std::endl;
	}

	bool bReturn = (out) ? true : false;
	out.close();
	return bReturn;
}




//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
namespace rms
{
template class PolyLine2<float>;
template class PolyLine2<double>;
}
//----------------------------------------------------------------------------
