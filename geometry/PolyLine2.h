// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#ifndef __RMS_WMLEXT_POLYLINE_2_H__
#define __RMS_WMLEXT_POLYLINE_2_H__

#include "config.h"
#include <Wm4Vector2.h>
#include <Wm4Matrix2.h>
#include <Wm4AxisAlignedBox2.h>

#include <vector>

namespace rms {

template<class Real>
class PolyLine2
{
public:

	PolyLine2();
	PolyLine2( const PolyLine2<Real> & copy );
	~PolyLine2();

	// add/remove vertex functions
	inline unsigned int AppendVertex( const Wml::Vector2<Real> & vVertex );
	inline void ClearVertices();

	// vertex count stuff
	inline unsigned int VertexCount() const;
	inline bool IsEmpty() const;

	// element access
	inline const Wml::Vector2<Real> & Vertex( unsigned int nVertex ) const;
	inline Wml::Vector2<Real> & Vertex( unsigned int nVertex );
	inline Wml::Vector2<Real> & operator[]( int i );
	inline const Wml::Vector2<Real> & operator[] (int i ) const;
	inline const std::vector< Wml::Vector2<Real> > & Vertices() const;

	// reverse order of vertices
	void Reverse();

	// get (unsigned) distance to polygon
	Real SqrDistance( const Wml::Vector2<Real> & vPoint, Wml::Vector2<Real> * pNearest = NULL );

	// bounding things
	void BoundingBox( Wml::AxisAlignedBox2<Real> & dest ) const;
	Real GetLength() const;

	//! param here is in range [0,1]
	Wml::Vector2<Real> Vertex( Real fArcLengthParam );

	// translation / rotation functions
	void Transform( const Wml::Matrix2<Real> & transform );
	void Translate( Real fX, Real fY );
	void Scale( Real fScaleX, Real fScaleY, const Wml::Vector2<Real> & vOrigin );

	// simplify polygon
	void Simplify( bool bSimplifyStraightLines = true );

	// save and load to file
	bool Load( const char * pFilename );
	bool Save( const char * pFilename ) const;

protected:
	std::vector< Wml::Vector2<Real> > m_vVertices;

	// support for arc-length parameterization
	Real m_fCacheLength;
	Real m_fCacheBucketStep;
	std::vector< Real > m_vVtxParams;
	std::vector< unsigned int > m_vParamTable;
	void PrecomputeArcLengthParams();
};

typedef PolyLine2<float> PolyLine2f;
typedef PolyLine2<double> PolyLine2d;


/*
 * inlines
 */

template<class Real>
inline unsigned int PolyLine2<Real>::AppendVertex( const Wml::Vector2<Real> & vVertex )
{
	size_t nVerts = m_vVertices.size();
	m_vVertices.push_back(vVertex);
	return (unsigned int)nVerts;
}

template<class Real>
inline void PolyLine2<Real>::ClearVertices( )
{
	m_vVertices.resize(0);
}

template<class Real>
inline unsigned int PolyLine2<Real>::VertexCount() const
{
	return (unsigned int)m_vVertices.size();
}

template<class Real>
inline bool PolyLine2<Real>::IsEmpty() const
{
	return m_vVertices.empty();
}

template<class Real>
inline const Wml::Vector2<Real> & PolyLine2<Real>::Vertex( unsigned int nVertex ) const
{
	return m_vVertices[nVertex];
}

template<class Real>
inline Wml::Vector2<Real> & PolyLine2<Real>::Vertex( unsigned int nVertex )
{
	return m_vVertices[nVertex];
}

template<class Real>
inline const Wml::Vector2<Real> & PolyLine2<Real>::operator[]( int i ) const
{
	return m_vVertices[i];
}

template<class Real>
inline Wml::Vector2<Real> & PolyLine2<Real>::operator[]( int i )
{
	return m_vVertices[i];
}

template<class Real>
inline const std::vector< Wml::Vector2<Real> > & PolyLine2<Real>::Vertices() const
{
	return m_vVertices;
}




} // namespace rms



#endif // __RMS_POLYLINE_2_H__