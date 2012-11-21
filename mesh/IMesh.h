// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#ifndef __RMS_ITRIANGLEMESH_3D__
#define __RMS_ITRIANGLEMESH_3D__
#include "config.h"
#include <vector>
#include <limits>
#include <Wm4Vector3.h>
#include <Wm4Vector2.h>
#include "SparseArray.h"

// useful elsewhere
#define IMESH_INVALID_ID std::numeric_limits<unsigned int>::max()


namespace rms
{


class IMesh
{
public:
	typedef unsigned int MeshEntityID;
	typedef MeshEntityID VertexID;
	typedef MeshEntityID EdgeID;
	typedef MeshEntityID TriangleID;
	typedef MeshEntityID UVSetID;
	typedef MeshEntityID ScalarSetID;

	static unsigned int InvalidID;

	IMesh();
	~IMesh();

/*
 * IMesh read interface (mandatory)
 */
	virtual void GetVertex( VertexID vID, Wml::Vector3f & vVertex, Wml::Vector3f * pNormal = NULL ) const = 0;
	virtual void GetNormal( VertexID vID, Wml::Vector3f & vNormal ) const = 0;
	virtual unsigned int GetVertexCount() const = 0;
	virtual unsigned int GetMaxVertexID() const = 0;
	virtual bool IsVertex( VertexID vID ) const = 0;

	virtual void GetTriangle( TriangleID tID, VertexID vTriangle[3]  ) const = 0;
	virtual void GetTriangle( TriangleID tID, Wml::Vector3f vTriangle[3], Wml::Vector3f * pNormals = NULL ) const = 0;
	virtual unsigned int GetTriangleCount() const = 0;
	virtual unsigned int GetMaxTriangleID() const = 0;
	virtual bool IsTriangle( TriangleID tID ) const = 0;

	//! neighbour tris are listed in order for edges [0,1],  [1,2],  [2,0]. Maybe be invalid, if no nbr
	virtual void FindNeighbours( TriangleID tID, TriangleID vNbrs[3] ) const;

/*
 *  IMesh write interface (optional)
 */
	virtual VertexID AppendVertex( const Wml::Vector3f & vVertex, const Wml::Vector3f * pNormal = NULL ) { return InvalidID; }

	virtual TriangleID AppendTriangle( VertexID v1, VertexID v2, VertexID v3 ) { return InvalidID; }
	virtual bool SetTriangle( TriangleID tID, VertexID v1, VertexID v2, VertexID v3 ) { return false; }

	virtual void Clear( bool bFreeMem ) {
		for ( unsigned int i = 0; i < m_vUVSets.size(); ++i )
			m_vUVSets[i]->Clear();
		//	if ( m_vUVSets[i] )
		//		delete m_vUVSets[i];
		//m_vUVSets.clear();
	}


	struct VtxNbrItr {
		VertexID vID;
		unsigned long long nData[2];
		VtxNbrItr() {}
		VtxNbrItr( VertexID id ) { vID = id; nData[0] = nData[1] = 0; }
	};

/*
 * IMesh mesh info interface - has default implementation
 */
	//! initialize vertex neighbour iteration
	virtual void BeginVtxTriangles( VtxNbrItr & v )  const {}

	//! (possibly) un-ordered iteration around one-ring of a vertex. Returns InvalidID when done
	virtual TriangleID GetNextVtxTriangle( VtxNbrItr & v ) const { return InvalidID; }

	//! determine if a vertex is on the mesh boundary (if there is one)  (depends on GetNextVtxTriangle)
	virtual bool IsBoundaryVertex( VertexID vID ) const;


/*
 * callback versions that actually work...
 */
	class NeighborTriCallback {
	public:
		virtual void BeginTriangles() {}
		virtual void NextTriangle( TriangleID tID ) = 0;
		virtual void EndTriangles() {}
	};

	virtual void NeighbourIteration( VertexID vID, NeighborTriCallback * pCallback );

/*
 * IMesh UV interface - has default implementation
 */
	inline virtual bool HasUVSet( UVSetID nSetID ) const
		{ return nSetID < m_vUVSets.size(); }
	inline virtual UVSetID AppendUVSet( ) 
		{ m_vUVSets.push_back( new UVSet() ); return (unsigned int)m_vUVSets.size()-1; }
	inline virtual void InitializeUVSet( UVSetID nSetID )
		{ m_vUVSets[nSetID]->Initialize(this); }
	inline virtual bool GetUV( VertexID vID, UVSetID nSetID, Wml::Vector2f & vUV ) const
		{ return m_vUVSets[nSetID]->GetUV(vID, vUV); }
	inline virtual bool GetTriangleUV( TriangleID tID, UVSetID nSetID, Wml::Vector2f vUV[3] ) const
		{ VertexID nTri[3]; GetTriangle(tID, nTri);
		  return m_vUVSets[nSetID]->GetUV(nTri, vUV); }
	inline virtual void SetUV( VertexID vID, UVSetID nSetID, const Wml::Vector2f & vUV )
		{ m_vUVSets[nSetID]->SetUV(vID, vUV); }
	inline virtual void AddUV( VertexID vID, UVSetID nSetID, const Wml::Vector2f & vUV )
		{ m_vUVSets[nSetID]->AddUV(vID, vUV); }
	inline virtual void ClearUVSet( UVSetID nSetID )
		{ m_vUVSets[nSetID]->Clear(); }

/*
 * IMesh iterator interface
 */
public:

	/*
	 * iterators
	 */
	class IVtxIterator {
	public:
		inline IVtxIterator( const IVtxIterator & i2 ) { 
			m_pMesh = i2.m_pMesh;
			m_itr = m_pMesh->ivtx_make_iterator( i2.m_itr ); }
		inline virtual ~IVtxIterator() 
			{ if ( m_itr ) m_pMesh->ivtx_free_iterator( m_itr ); }
		inline const IVtxIterator & operator=( const IVtxIterator & i2 )
			{ m_pMesh->ivtx_set(m_itr, i2.m_itr); return *this; }
		
		inline VertexID operator*() 
			{ return m_pMesh->ivtx_value( m_itr ); }

		//! postfix operator doesn't return value, avoids memory allocation for copy
		inline void operator++(int)  // postfix
			{ m_pMesh->ivtx_goto_next(m_itr); }
		inline IVtxIterator & operator++() // prefix
			{ m_pMesh->ivtx_goto_next(m_itr); return *this; }

		inline bool operator==( const IVtxIterator & i2 ) 
			{ return m_pMesh->ivtx_equal( m_itr, i2.m_itr ); }
		inline bool operator!=( const IVtxIterator & i2 ) 
			{ return ! m_pMesh->ivtx_equal( m_itr, i2.m_itr ); }

	protected:
		const IMesh * m_pMesh;
		void * m_itr;
	
		//! if bStart, return start iterator, else return end iterator
		inline IVtxIterator(const IMesh * pMesh, bool bStart) : m_pMesh(pMesh)
			{ m_itr = m_pMesh->ivtx_make_iterator(bStart); }
		friend class IMesh;
	};


	class ITriIterator {
	public:
		inline ITriIterator( const ITriIterator & i2 ) {
			m_pMesh = i2.m_pMesh;
			m_itr = m_pMesh->itri_make_iterator( i2.m_itr ); }
		inline virtual ~ITriIterator() { 
			if ( m_itr ) m_pMesh->itri_free_iterator( m_itr ); }
		inline const ITriIterator & operator=( const ITriIterator & i2 ) { 
			m_pMesh->itri_set(m_itr, i2.m_itr); return *this; }
		
		inline TriangleID operator*() 
			{ return m_pMesh->itri_value( m_itr ); }

		//! postfix operator doesn't return value, avoids memory allocation for copy
		inline void operator++(int)  // postfix
			{ m_pMesh->itri_goto_next(m_itr); }
		inline ITriIterator & operator++() // prefix
			{ m_pMesh->itri_goto_next(m_itr); return *this; }

		inline bool operator==( const ITriIterator & i2 ) 
			{ return m_pMesh->itri_equal( m_itr, i2.m_itr ); }
		inline bool operator!=( const ITriIterator & i2 ) 
			{ return ! m_pMesh->itri_equal( m_itr, i2.m_itr ); }

	protected:
		const IMesh * m_pMesh;
		void * m_itr;
	
		//! if bStart, return start iterator, else return end iterator
		inline ITriIterator(const IMesh * pMesh, bool bStart) : m_pMesh(pMesh)
			{ m_itr = m_pMesh->itri_make_iterator(bStart); }
		friend class IMesh;
	};


	inline IVtxIterator BeginIVertices() const  { return IVtxIterator(this, true); }
	inline IVtxIterator EndIVertices() const    { return IVtxIterator(this, false); }

	inline ITriIterator BeginITriangles() const { return ITriIterator(this, true); }
	inline ITriIterator EndITriangles() const   { return ITriIterator(this, false); }

protected:
/*
 * IMesh iterator interface (mandatory)
 */
	virtual void * ivtx_make_iterator(bool bStart) const = 0;
	virtual void * ivtx_make_iterator( void * pFromIterator ) const = 0;
	virtual void ivtx_set( void * pIterator, void * pToIteratorVal ) const = 0;
	virtual void ivtx_free_iterator( void * pIterator ) const = 0;
	virtual void ivtx_goto_next( void * pIterator ) const = 0;
	virtual bool ivtx_equal( void * pIterator1, void * pIterator2 ) const = 0;
	virtual VertexID ivtx_value( void * pIterator ) const = 0;

	virtual void * itri_make_iterator(bool bStart) const = 0;
	virtual void * itri_make_iterator( void * pFromIterator )const  = 0;
	virtual void itri_set( void * pIterator, void * pToIteratorVal )const  = 0;
	virtual void itri_free_iterator( void * pIterator ) const = 0;
	virtual void itri_goto_next( void * pIterator ) const = 0;
	virtual bool itri_equal( void * pIterator1, void * pIterator2 )const  = 0;
	virtual TriangleID itri_value( void * pIterator ) const = 0;




public:
/*
 * UVSet class
 *
 * Note: There are two ways to use this class. One is to call Initialize() once your mesh
 *  has all its vertices, then call SetUV() afterwards. This is the most efficient in terms
 *  of memory allocation. Alternately, you can call Initialize() at any time (ie before all
 *  vertices are added), and then call AddUV() afterwards, and that will force a resize if the
 *  set is too small.
 * 
 */
	class UVSet {
	public:
		UVSet() {}
		UVSet( IMesh * pMesh )
			{ Initialize(pMesh); }

		inline void Initialize( IMesh * pMesh )
			{ Clear(); m_vUV.resize(pMesh->GetMaxVertexID()); }

		inline void Clear()
			{ m_vUV.clear(false); }

		inline void SetUV( VertexID vID, const Wml::Vector2f & vUV ) 
			{ m_vUV.set(vID, vUV); }

		inline void AddUV( VertexID vID, const Wml::Vector2f & vUV )
			{ m_vUV.set_and_grow(vID, vUV); }
		
		inline bool HasUV( VertexID vID ) const
			{ return m_vUV.has(vID); }

		inline bool GetUV( VertexID vID, Wml::Vector2f & vUV ) const
			{ if ( m_vUV.has(vID) )
				{ vUV = m_vUV[vID]; return true; } 
			  return false; 
			}
		
		bool GetUV( VertexID vID[3], Wml::Vector2f vUV[3] ) const
			{ return GetUV(vID[0],vUV[0]) && GetUV(vID[1],vUV[1]) && GetUV(vID[2],vUV[2]); }

		SparseArray<Wml::Vector2f> & UV() { return m_vUV; }

	protected:
		SparseArray<Wml::Vector2f> m_vUV;
	};

	UVSet & GetUVSet( UVSetID setID ) { return * m_vUVSets[setID]; }
	
protected:
	std::vector< UVSet * > m_vUVSets;



public:
/*
 * IMesh ScalarSet interface - has default implementation
 */
	inline virtual bool HasScalarSet( ScalarSetID nSetID ) const
		{ return nSetID < m_vScalarSets.size(); }
	inline virtual ScalarSetID AppendScalarSet( ) 
		{ m_vScalarSets.push_back( new ScalarSet() ); return (unsigned int)m_vScalarSets.size()-1; }
	inline virtual void InitializeScalarSet( ScalarSetID nSetID )
		{ m_vScalarSets[nSetID]->Initialize(this); }
	inline virtual bool GetScalar( VertexID vID, ScalarSetID nSetID, float & fValue ) const
		{ return m_vScalarSets[nSetID]->GetScalar(vID, fValue); }
	inline virtual bool GetTriangleScalar( TriangleID tID, ScalarSetID nSetID, float vScalar[3] ) const
		{ VertexID nTri[3]; GetTriangle(tID, nTri);
		  return m_vScalarSets[nSetID]->GetScalar(nTri, vScalar); }
	inline virtual void SetScalar( VertexID vID, ScalarSetID nSetID, float fValue )
		{ m_vScalarSets[nSetID]->SetScalar(vID, fValue); }
	inline virtual void ClearScalarSet( ScalarSetID nSetID )
		{ m_vScalarSets[nSetID]->Clear(); }


/*
 * ScalarSet class
 */
	class ScalarSet {
	public:
		ScalarSet() {}
		ScalarSet( IMesh * pMesh )
			{ Initialize(pMesh); }

		inline void Initialize( IMesh * pMesh )
			{ Clear(); m_vValue.resize(pMesh->GetMaxVertexID()); }

		inline void Clear()
			{ m_vValue.clear(false); }

		inline void SetScalar( VertexID vID, float fValue ) 
			{ m_vValue.set(vID, fValue); }
		
		inline bool HasScalar( VertexID vID ) const
			{ return m_vValue.has(vID); }

		inline bool GetScalar( VertexID vID, float & fValue ) const
			{ if ( m_vValue.has(vID) )
				{ fValue = m_vValue[vID]; return true; } 
			  return false; 
			}
		
		bool GetScalar( VertexID vID[3], float fValue[3] ) const
			{ return GetScalar(vID[0],fValue[0]) && GetScalar(vID[1],fValue[1]) && GetScalar(vID[2],fValue[2]); }

	protected:
		SparseArray<float> m_vValue;
	};

	ScalarSet & GetScalarSet( ScalarSetID setID ) { return * m_vScalarSets[setID]; }
	
protected:
	std::vector< ScalarSet * > m_vScalarSets;

};





/*
 * Functions on meshes
 */

template<class Real>
class IVertexFunction
{
public:
	IVertexFunction(rms::IMesh * pMesh)
		{ m_pMesh = pMesh; }

	rms::IMesh * Mesh() { return pMesh; }
	const rms::IMesh * Mesh() const { return pMesh; }
	
	virtual unsigned int Components() const = 0;
	virtual Real operator()(unsigned int vID, int nComponent) const = 0;
	virtual Real & operator()(unsigned int vID, int nComponent) = 0;

protected:
	rms::IMesh * m_pMesh;
};
typedef IVertexFunction<float> IVertexFunctionf;
typedef IVertexFunction<double> IVertexFunctiond;


template<class Real>
class VertexFunction1 : public IVertexFunction<Real>, public std::vector<Real>
{
public:
	VertexFunction1(rms::IMesh * pMesh) : IVertexFunction(pMesh)
		{ resize(pMesh->GetMaxVertexID()); }

	virtual unsigned int Components() const 
		{ return 1; }
	virtual Real operator()(unsigned int vID, int nComponent) const
		{ return (*this)[vID]; }
	virtual Real & operator()(unsigned int vID, int nComponent)
		{ return (*this)[vID]; }
};
typedef VertexFunction1<float> VertexFunction1f;
typedef VertexFunction1<double> VertexFunction1d;


template<class Real, int N>
class VertexFunction : public IVertexFunction<Real>
{
public:
	VertexFunction(rms::IMesh * pMesh) : IVertexFunction(pMesh) 
		{ m_vValues.resize(pMesh->GetMaxVertexID()); }
 
	virtual unsigned int Components() const 
		{ return N; }
	virtual Real operator()(unsigned int vID, unsigned int elem) const
		{ return m_vValues[vID].d[elem]; }
	virtual Real & operator()(unsigned int vID, unsigned int elem)
		{ return m_vValues[vID].d[elem]; }

protected:
	struct data {		// internal assignable array
		Real d[N];
	};
	std::vector<data> m_vValues;
};




template<class Real>
class ITriangleFunction
{
public:
	ITriangleFunction(rms::IMesh * pMesh)
		{ m_pMesh = pMesh; }

	rms::IMesh * Mesh() { return pMesh; }
	const rms::IMesh * Mesh() const { return pMesh; }
	
	virtual unsigned int Components() const = 0;
	virtual Real operator()(unsigned int vID, int nComponent) const = 0;
	virtual Real & operator()(unsigned int vID, int nComponent) = 0;

protected:
	rms::IMesh * m_pMesh;
};
typedef ITriangleFunction<float> ITriangleFunctionf;
typedef ITriangleFunction<double> ITriangleFunctiond;


template<class Real>
class TriangleFunction1 : public ITriangleFunction<Real>, public std::vector<Real>
{
public:
	TriangleFunction1(rms::IMesh * pMesh) : ITriangleFunction(pMesh)
		{ if ( m_pMesh ) resize(m_pMesh->GetMaxTriangleID()); }

	virtual unsigned int Components() const 
		{ return 1; }
	virtual Real operator()(unsigned int vID, int nComponent = 0) const
		{ return (*this)[vID]; }
	virtual Real & operator()(unsigned int vID, int nComponent = 0)
		{ return (*this)[vID]; }
};
typedef TriangleFunction1<float> TriangleFunction1f;
typedef TriangleFunction1<double> TriangleFunction1d;


template<class Real, int N>
class TriangleFunction : public ITriangleFunction<Real>
{
public:
	TriangleFunction(rms::IMesh * pMesh) : ITriangleFunction(pMesh) 
		{ if ( m_pMesh ) resize(m_pMesh->GetMaxTriangleID()); }
 
	virtual unsigned int Components() const 
		{ return N; }
	virtual Real operator()(unsigned int vID, unsigned int elem) const
		{ return m_vValues[vID].d[elem]; }
	virtual Real & operator()(unsigned int vID, unsigned int elem)
		{ return m_vValues[vID].d[elem]; }

protected:
	struct data {		// internal assignable array
		Real d[N];
	};
	std::vector<data> m_vValues;
};






//
//template <class Real, int N>
//VertexFunction<Real, N>::VertexFunction(rms::VFTriangleMesh * pMesh)
//{
//	m_pMesh = pMesh;
//	m_vValues.resize( pMesh->GetMaxVertexID() );
//}
//template <class Real, int N>
//Real VertexFunction<Real, N>::operator()(unsigned int vID, unsigned int elem) const
//{
//	
//}
//template <class Real, int N>
//Real & VertexFunction<Real, N>::operator()(unsigned int vID, unsigned int elem)
//{
//	return m_vValues[vID].d[elem];
//}
//
//
//typedef VertexFunction<float,1> VertexFunctionf;
//typedef VertexFunction<double,1> VertexFunctiond;








} // end namespace rms


#endif  // __RMS_ITRIANGLEMESH_3D__