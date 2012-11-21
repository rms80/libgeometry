// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#ifndef __RMS_WMLEXT_LINEARCOMPLEX_2_H__
#define __RMS_WMLEXT_LINEARCOMPLEX_2_H__

#include "config.h"
#include "WmlPolygon2.h"

namespace rms {


// [RMS TODO: this class could contain optimization stuff like the polygon
//   containment ordering, faster inside/outside test, etc...
template<class Real>
class PolygonSet2
{
public:
	PolygonSet2() {}
	~PolygonSet2() {}

	void Clear() 
		{ m_vPolygons.resize(0); }

	Polygon2<Real> & AppendPolygon()
		{ m_vPolygons.resize( m_vPolygons.size() + 1 ); return m_vPolygons.back(); }
	void AddPolygon( const Polygon2<Real> & vPoly )
		{ m_vPolygons.push_back(vPoly); }

	unsigned int PolygonCount() 
		{ return (unsigned int)m_vPolygons.size(); }
	Polygon2<Real> & Polygon( unsigned int i ) 
		{ return m_vPolygons[i]; }
	const Polygon2<Real> & Polygon( unsigned int i ) const
		{ return m_vPolygons[i]; }

	Polygon2<Real> & operator[]( unsigned int i ) 
		{ return m_vPolygons[i]; }

	void Translate( Real fX, Real fY );
	void Scale( Real fScaleX, Real fScaleY, const Wml::Vector2<Real> & vOrigin );

	void BoundingBox( Wml::AxisAlignedBox2<Real> & dest ) const;

protected:
	std::vector< Polygon2<Real> > m_vPolygons;
};


typedef PolygonSet2<float> PolygonSet2f;
typedef PolygonSet2<double> PolygonSet2d;



template<class Real>
class LinearComplex2
{
public:
	
	typedef unsigned int LCVertex;
	struct LCEdge {
		LCVertex v1;
		LCVertex v2;
		LCEdge() { v1 = v2 = 0; }
		LCEdge( LCVertex nV1, LCVertex nV2 ) { v1 = nV1; v2 = nV2; }
	};

	LinearComplex2();
	LinearComplex2( const LinearComplex2<Real> & copy );
	~LinearComplex2();

	inline void Clear();

	// add/remove vertex functions
	inline unsigned int AppendVertex( const Wml::Vector2<Real> & vVertex );

	// vertex count stuff
	inline unsigned int VertexCount() const;
	inline bool IsEmpty() const;

	// element access
	inline const Wml::Vector2<Real> & Vertex( unsigned int nVertex ) const;
	inline Wml::Vector2<Real> & Vertex( unsigned int nVertex );
	inline Wml::Vector2<Real> & operator[]( int i );
	inline const Wml::Vector2<Real> & operator[] (int i ) const;
	inline const std::vector< Wml::Vector2<Real> > & Vertices() const;

	// add/remove edge functions
	inline unsigned int AppendEdge( const LCEdge & edge );

	inline unsigned int EdgeCount();
	inline const LCEdge & Edge( unsigned int nEdge );
	inline const std::vector< LCEdge > & Edges(); 

	bool IsValidEdge( LCEdge & edge );
	int GetNeighbourCount( unsigned int nVertex );

	// returns true if all vertices have 2 neighbors
	bool IsClosed();

	// returns true if all vertices have at most 2 neighbors
	bool IsSimple();

	// bounding things
	void BoundingBox( Wml::AxisAlignedBox2<Real> & dest ) const;
	Wml::Vector2<Real> CenterOfMass() const;

	// translation / rotation functions
	void Transform( const Wml::Matrix2<Real> & transform );
	void Translate( Real fX, Real fY );

	// decompose into polygons
	bool Decompose( PolygonSet2<Real> & loopSet, bool bSimplify = false );

	// OpenGL drawing routines
	void DrawGL();

protected:
	std::vector< Wml::Vector2<Real> > m_vVertices;
	std::vector< LCEdge > m_vEdges;
};

typedef LinearComplex2<float> LinearComplex2f;
typedef LinearComplex2<double> LinearComplex2d;


/*
 * inlines
 */

template<class Real>
inline unsigned int LinearComplex2<Real>::AppendVertex( const Wml::Vector2<Real> & vVertex )
{
	size_t nVerts = m_vVertices.size();
	m_vVertices.push_back(vVertex);
	return (unsigned int)nVerts;
}

template<class Real>
inline void LinearComplex2<Real>::Clear( )
{
	m_vVertices.resize(0);
	m_vEdges.resize(0);
}

template<class Real>
inline unsigned int LinearComplex2<Real>::VertexCount() const
{
	return (unsigned int)m_vVertices.size();
}


template<class Real>
inline const Wml::Vector2<Real> & LinearComplex2<Real>::Vertex( unsigned int nVertex ) const
{
	return m_vVertices[nVertex];
}

template<class Real>
inline Wml::Vector2<Real> & LinearComplex2<Real>::Vertex( unsigned int nVertex )
{
	return m_vVertices[nVertex];
}

template<class Real>
inline const Wml::Vector2<Real> & LinearComplex2<Real>::operator[]( int i ) const
{
	return m_vVertices[i];
}

template<class Real>
inline Wml::Vector2<Real> & LinearComplex2<Real>::operator[]( int i )
{
	return m_vVertices[i];
}

template<class Real>
inline const std::vector< Wml::Vector2<Real> > & LinearComplex2<Real>::Vertices() const
{
	return m_vVertices;
}


template<class Real>
inline unsigned int LinearComplex2<Real>::AppendEdge( typename const LinearComplex2<Real>::LCEdge & edge )
{
	size_t nEdge = m_vEdges.size();
	m_vEdges.push_back( edge );
	return (unsigned int)nEdge;
}

template<class Real>
inline unsigned int LinearComplex2<Real>::EdgeCount()
{
	return (unsigned int)m_vEdges.size();
}

template<class Real>
inline typename const LinearComplex2<Real>::LCEdge & LinearComplex2<Real>::Edge( unsigned int nEdge )
{
	return m_vEdges[nEdge];
}

template<class Real>
inline const std::vector< typename LinearComplex2<Real>::LCEdge > & LinearComplex2<Real>::Edges()
{
	return m_vEdges;
}



template<class Real>
inline bool LinearComplex2<Real>::IsEmpty() const
{
	return m_vVertices.empty() || m_vEdges.empty();
}



} // namespace rmssketch



#endif // __RMS_WMLEXT_LINEARCOMPLEX_2_H__