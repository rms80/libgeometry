// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#include "LinearComplex2.h"
#include <VectorUtil.h>
#include <opengl.h>
#include <limits>

using namespace rms;

template <class Real>
void PolygonSet2<Real>::Translate( Real fX, Real fY )
{
	size_t nCount = m_vPolygons.size();
	for ( unsigned int i = 0; i < nCount; ++i )
		m_vPolygons[i].Translate( fX, fY );
}

template <class Real>
void PolygonSet2<Real>::Scale( Real fScaleX, Real fScaleY, const Wml::Vector2<Real> & vOrigin )
{
	size_t nCount = m_vPolygons.size();
	for ( unsigned int i = 0; i < nCount; ++i ) 
		m_vPolygons[i].Scale(fScaleX, fScaleY, vOrigin);
}

template <class Real>
void PolygonSet2<Real>::BoundingBox( Wml::AxisAlignedBox2<Real> & dest ) const
{
	if ( m_vPolygons.empty() )
		return;
	m_vPolygons[0].BoundingBox( dest );
	size_t nCount = m_vPolygons.size();
	for ( unsigned int i = 1; i < nCount; ++i ) {
		Wml::AxisAlignedBox2<Real> box2;
		m_vPolygons[i].BoundingBox(box2);
		rms::Union( dest, box2 );
	}
}



template <class Real>
LinearComplex2<Real>::LinearComplex2()
{
}

template <class Real>
LinearComplex2<Real>::LinearComplex2( const LinearComplex2<Real> & copy )
{
	m_vVertices = copy.m_vVertices;
	m_vEdges = copy.m_vEdges;
}

template <class Real>
LinearComplex2<Real>::~LinearComplex2()
{
}
template <class Real>
bool LinearComplex2<Real>::IsValidEdge( LCEdge & edge )
{
	size_t nEdges = m_vEdges.size();
	for (unsigned int i = 0; i < nEdges; ++i) {

		LCEdge & curEdge = m_vEdges[i];

		if ( (curEdge.v1 == edge.v1 && curEdge.v2 == edge.v2) ||
			 (curEdge.v1 == edge.v2 && curEdge.v2 == edge.v1) )
			 return false;
	}

	return true;
}


template <class Real>
int LinearComplex2<Real>::GetNeighbourCount( unsigned int nVertex )
{
	int nConnected = 0;
	size_t nEdges = m_vEdges.size();
	for (unsigned int i = 0; i < nEdges; ++i) {

		LCEdge & curEdge = m_vEdges[i];

		if ( curEdge.v1 == nVertex || curEdge.v2 == nVertex )
			++nConnected;
	}

	return nConnected;
}


// returns true if all vertices have 2 neighbors
template <class Real>
bool LinearComplex2<Real>::IsClosed()
{
	std::vector<unsigned char> vVtxCounts;
	vVtxCounts.assign( m_vVertices.size(), 0 );

	size_t nEdges = m_vEdges.size();
	for (unsigned int i = 0; i < nEdges; ++i) {
		LCEdge & edge = m_vEdges[i];

		vVtxCounts[ edge.v1 ]++;
		vVtxCounts[ edge.v2 ]++;
	}

	size_t nVertices = vVtxCounts.size();
	for (unsigned int i = 0; i < nVertices; ++i)
		if (vVtxCounts[i] < 2)
			return false;
	return true;
}

// returns true if all vertices have at most 2 neighbors
template <class Real>
bool LinearComplex2<Real>::IsSimple()
{
	std::vector<unsigned char> vVtxCounts;
	vVtxCounts.assign( m_vVertices.size(), 0 );

	size_t nEdges = m_vEdges.size();
	for (unsigned int i = 0; i < nEdges; ++i) {
		LCEdge & edge = m_vEdges[i];

		vVtxCounts[ edge.v1 ]++;
		vVtxCounts[ edge.v2 ]++;

		if (vVtxCounts[ edge.v1 ] > 2 || vVtxCounts[ edge.v2 ] > 2)
			return false;
	}
	return true;
}





// bounding things
template <class Real>
void LinearComplex2<Real>::BoundingBox( Wml::AxisAlignedBox2<Real> & box ) const
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
Wml::Vector2<Real> LinearComplex2<Real>::CenterOfMass() const
{
	Wml::Vector2<Real> vSum( Wml::Vector2<Real>::ZERO );
	size_t nVertices = m_vVertices.size();
	for (unsigned int i = 1; i < nVertices; ++i)
		vSum += m_vVertices[i];
	if ( nVertices > 0 )
		vSum *= 1.0f / (float)nVertices;
	return vSum;
}



// translation / rotation functions
template <class Real>
void LinearComplex2<Real>::Transform( const Wml::Matrix2<Real> & transform )
{
	size_t nCount = m_vVertices.size();
	for (unsigned int i = 0; i < nCount; ++i)
		m_vVertices[i] = transform * m_vVertices[i];
}

template <class Real>
void LinearComplex2<Real>::Translate( Real fX, Real fY )
{
	size_t nCount = m_vVertices.size();
	for (unsigned int i = 0; i < nCount; ++i) {
		m_vVertices[i].X() += fX;
		m_vVertices[i].Y() += fY;
	}
}



template <class Real>
bool LinearComplex2<Real>::Decompose( PolygonSet2<Real> & loopSet, bool bSimplify )
{
	size_t nVertexCount = m_vVertices.size();
	size_t nEdgeCount = m_vEdges.size();
	if (nVertexCount  < 3 || nEdgeCount < 3)
		return false;

	// make bitmap for marking vertices
	std::vector<bool> vMarkedVertices;
	vMarkedVertices.resize( nVertexCount );
	vMarkedVertices.assign( nVertexCount, false );

	bool bFinished = false;
	while (!bFinished) {

		Polygon2<Real> newPoly;

		// search for an unmarked vertex
		unsigned int nCurVertex = std::numeric_limits<unsigned int>::max();
		for (unsigned int i = 0; i < nVertexCount; ++i) {
			if (vMarkedVertices[i] == false) {
				nCurVertex = i;
				break;
			}
		}
		if ( nCurVertex == std::numeric_limits<unsigned int>::max() ) {
			bFinished = true;
			continue;
		}
		
		// now follow this loop until we run out of vertices
		bool bStuck = false;
		while ( ! bStuck ) {

			// append vertex to current poly and mark it as done
			newPoly.AppendVertex( m_vVertices[nCurVertex] );
			vMarkedVertices[nCurVertex] = true;

			// find edges connected to this vertex
			int vi = 0;
			int nConnectedVtx[3];
			for (unsigned int i = 0; i < nEdgeCount && vi < 3; ++i) {
				LCEdge & edge = m_vEdges[i];
				if ( (edge.v1 == nCurVertex) )
					nConnectedVtx[vi++] = edge.v2;
				else if ( (edge.v2 == nCurVertex) )
					nConnectedVtx[vi++] = edge.v1;
			}

			if ( vi != 2 ) {		// vertex with connectivity > 2. We're hosed...
				bStuck = true;
                                _RMSInfo("Connectivity error in GetLoops!\n");
				continue;
			}

			// pick vertex in direction we have not seen
			if ( vMarkedVertices[nConnectedVtx[0]] == false )
				nCurVertex = nConnectedVtx[0];
			else if ( vMarkedVertices[nConnectedVtx[1]] == false )
				nCurVertex = nConnectedVtx[1];
			else
				bStuck = true;		// seen both directions. must be finished!
		}

		// ok, we got stuck. If the poly is empty, assume we are finished.
		// if it only has a few vertices, don't add it. otherwise continue.
		if ( newPoly.IsEmpty() ) {
			bFinished = true;
		} else if ( newPoly.VertexCount() > 3 ) {
			if (newPoly.IsClockwise() == true)
				newPoly.FlipOrientation();
			loopSet.AddPolygon( newPoly );
		}
	}

	// only return true if we marked all vertices
	for (unsigned int i = 0; i < nVertexCount; ++i)
		if ( vMarkedVertices[i] == false) 
			return false;

	if ( bSimplify ) {
		size_t nLoops = loopSet.PolygonCount();

		// simplify the loops to speed up inside/outside testing..
		for (unsigned int k = 0; k < nLoops; ++k)
			loopSet[k].Simplify(false);

	}

	return true;
}


template <class Real>
void LinearComplex2<Real>::DrawGL()
{
	PolygonSet2<Real> polys;
	if ( Decompose(polys) != false) {

		unsigned int nLoops = polys.PolygonCount();
		for ( unsigned int i = 0; i < nLoops; ++i) {
			Polygon2<Real> poly = polys[i];
			size_t nVerts = poly.VertexCount();
			glBegin(GL_LINE_LOOP);
			for ( unsigned int j = 0; j < nVerts; ++j) {
				glVertex2f( (float)poly[j].X(), (float)poly[j].Y() );
			}
			glEnd();

			//glPointSize(5.0f);
			//glColor3f(0,1,0);
			//glBegin(GL_POINTS);
			//for (unsigned int j = 0; j < nVerts; ++j) {
			//	glVertex2f( poly[j].X(), poly[j].Y() );
			//}
			//glEnd();

			//poly.Simplify();
			//nVerts = poly.VertexCount();
	
			//glPointSize(3.0f);
			//glColor3f(0,0,1);
			//glBegin(GL_POINTS);
			//for (unsigned int j = 0; j < nVerts; ++j) {
			//	glVertex2f( poly[j].X(), poly[j].Y() );
			//}
			//glEnd();
		}

	} else {

		glBegin(GL_LINES);
		size_t nEdges = m_vEdges.size();
		for (unsigned int i = 0; i < nEdges; ++i) {
			LCEdge & edge = m_vEdges[i];
			glVertex2f( (float) m_vVertices[edge.v1].X(), (float)m_vVertices[edge.v1].Y() );
			glVertex2f( (float) m_vVertices[edge.v2].X(), (float)m_vVertices[edge.v2].Y() );
		}
		glEnd();
	}
}



//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
namespace rms
{
template class PolygonSet2<float>;
template class PolygonSet2<double>;
template class LinearComplex2<float>;
template class LinearComplex2<double>;
}
//----------------------------------------------------------------------------
