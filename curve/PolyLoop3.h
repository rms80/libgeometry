// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#ifndef __RMS_POLYLOOP3_H__
#define __RMS_POLYLOOP3_H__
#include "config.h"
#include <Wm4Vector3.h>
#include <Wm4Matrix3.h>
#include <Wm4AxisAlignedBox3.h>
#include <Wm4ColorRGBA.h>

#include <vector>

namespace rms {

class IPolyCurve
{
public:
	typedef unsigned int VertexID;
	static VertexID InvalidID;

	virtual void GetNeighbours( VertexID nVertex, VertexID & nLeft, VertexID & nRight ) const = 0;

};


template<class Real>
class PolyLoop3 : public IPolyCurve
{
public:

	PolyLoop3();
	PolyLoop3( const PolyLoop3<Real> & copy );
	~PolyLoop3();

	// add/remove vertex functions
	inline unsigned int AppendVertex( const Wml::Vector3<Real> & vVertex );
	inline void AppendVertices( const std::vector<Wml::Vector3<Real> > & vVerts );
	inline void Clear();

	// vertex count stuff
	inline unsigned int VertexCount() const;
	inline bool IsEmpty() const;

	// element access
	inline const Wml::Vector3<Real> & Vertex( VertexID nVertex ) const;
	inline Wml::Vector3<Real> & Vertex( VertexID nVertex );
	inline Wml::Vector3<Real> & operator[]( VertexID i );
	inline const Wml::Vector3<Real> & operator[] ( VertexID i ) const;
	inline const std::vector< Wml::Vector3<Real> > & Vertices() const;

	inline void SetVertex( VertexID nVertex, const Wml::Vector3<Real> & vPosition );

	inline virtual void GetNeighbours( VertexID nVertex, VertexID & nLeft, VertexID & nRight ) const;

	void FlipOrientation();

	// get (unsigned) distance to polygon
	Real SqrDistance( const Wml::Vector3<Real> & vPoint, Wml::Vector3<Real> * pNearest = NULL );

	// bounding things
	void BoundingBox( Wml::AxisAlignedBox3<Real> & dest ) const;
	Wml::Vector3<Real> Centroid() const;

	// translation / rotation functions
	void Transform( const Wml::Matrix3<Real> & transform, const Wml::Vector3<Real> & vOrigin );
	void Translate( Real fX, Real fY, Real fZ );
	void Scale( Real fScaleX, Real fScaleY, Real fScaleZ, const Wml::Vector3<Real> & vOrigin );

	// simplify polygon
	void Simplify( bool bSimplifyStraightLines = true );

	// smooth out polygon and then subdivide 
	void SmoothResample( float fFactor );

	void Render( bool bLines, bool bPoints = false );


protected:
	std::vector< Wml::Vector3<Real> > m_vVertices;
};

typedef PolyLoop3<float> PolyLoop3f;
typedef PolyLoop3<double> PolyLoop3d;






/*
 * inlines
 */

template<class Real>
inline unsigned int PolyLoop3<Real>::AppendVertex( const Wml::Vector3<Real> & vVertex )
{
	size_t nVerts = m_vVertices.size();
	m_vVertices.push_back(vVertex);
	return (unsigned int)nVerts;
}

template<class Real>
inline void PolyLoop3<Real>::AppendVertices( const std::vector<Wml::Vector3<Real> > & vVerts )
{
	m_vVertices = vVerts;
}


template<class Real>
inline void PolyLoop3<Real>::Clear( )
{
	m_vVertices.resize(0);
}

template<class Real>
inline unsigned int PolyLoop3<Real>::VertexCount() const
{
	return (unsigned int)m_vVertices.size();
}

template<class Real>
inline bool PolyLoop3<Real>::IsEmpty() const
{
	return m_vVertices.empty();
}

template<class Real>
inline const Wml::Vector3<Real> & PolyLoop3<Real>::Vertex( VertexID nVertex ) const
{
	lgASSERT( nVertex < m_vVertices.size() );
	return m_vVertices[nVertex];
}

template<class Real>
inline Wml::Vector3<Real> & PolyLoop3<Real>::Vertex( VertexID nVertex )
{
	lgASSERT( nVertex < m_vVertices.size() );
	return m_vVertices[nVertex];
}

template<class Real>
inline const Wml::Vector3<Real> & PolyLoop3<Real>::operator[]( VertexID i ) const
{
	lgASSERT( i < m_vVertices.size() );
	return m_vVertices[i];
}

template<class Real>
inline Wml::Vector3<Real> & PolyLoop3<Real>::operator[]( VertexID i )
{
	lgASSERT( i < m_vVertices.size() );
	return m_vVertices[i];
}

template<class Real>
inline const std::vector< Wml::Vector3<Real> > & PolyLoop3<Real>::Vertices() const
{
	return m_vVertices;
}

template<class Real>
inline void PolyLoop3<Real>::SetVertex( VertexID nVertex, const Wml::Vector3<Real> & vPosition )
{
	lgASSERT( nVertex < m_vVertices.size() );
	m_vVertices[nVertex] = vPosition;
}

template<class Real>
inline void PolyLoop3<Real>::GetNeighbours( VertexID nVertex, VertexID & nLeft, VertexID & nRight ) const
{
	unsigned int nMax = (unsigned int)m_vVertices.size();
	if ( nVertex == 0 ) {
		nLeft = nMax-1;
		nRight = 1;
	} else {
		nLeft = nVertex - 1;
		nRight = (nVertex+1) % nMax;
	}
}



} // namespace rms



#endif // __RMS_POLYLOOP3_H__