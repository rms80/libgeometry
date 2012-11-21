// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#include "config.h"
#include <VFTriangleMesh.h>
#include <IMeshBVTree.h>
#include <Wm4AxisAlignedBox2.h>
#include <Wm4Matrix3.h>
#include "IGeometry.h"
#include "NeighbourCache.h"
#include "MeshSelection.h"
#include "IDMap.h"

namespace rms {

class MeshUtils
{
public:

	static void CopyMesh( IMesh * pFrom, IMesh * pTo, VertexMap * pVtxMap = NULL, TriangleMap * pFaceMap = NULL, bool bCompact = true );	

	static void GetSubMesh( VFTriangleMesh & original, VFTriangleMesh & submesh, 
							const BitSet & vTris,
							VertexMap * pVtxMap = NULL,	TriangleMap * pFaceMap = NULL, bool bCopyUVs = false);
	static void GetSubMesh( VFTriangleMesh & original, VFTriangleMesh & submesh, 
							const std::vector<IMesh::TriangleID> & vTris,
							VertexMap * pVtxMap = NULL,	TriangleMap * pFaceMap = NULL, bool bCopyUVs = false);

	//! coord map would be [0,1] to, for example, map UV to XY, or [0,2] to map UV to XZ
	static void SetMeshXYZtoUV( VFTriangleMesh & mesh, const Wml::Vector3f & vNormal, int nCoordMap[2],  
		std::vector<Wml::Vector3f> * pvXYZPositions = NULL, std::vector<Wml::Vector3f> * pvXYZNormals = NULL );

	static void CopyMeshXYZtoUV( VFTriangleMesh & mesh, int nCoordMap[2], IMesh::UVSetID nSetID );

	//! assumes that from & to are same mesh (must have same VIDs). Will create toSetID if necessary.
	static void CopyUVs( const VFTriangleMesh & from, VFTriangleMesh & to, IMesh::UVSetID nFromSetID, IMesh::UVSetID nToSetID, bool bBoundaryOnly = false);

	//! copy UVs from one set to another
	static void CopyUVs( IMesh::UVSet & from, IMesh::UVSet & to, rms::VertexMap * pVtxMap = NULL );


	//! vVertexMap maps 'from' verts to 'to' verts
	static void CopyUVs( const IMesh & from, VFTriangleMesh & to, IMesh::UVSetID nFromSetID, IMesh::UVSetID nToSetID,
		const VertexMap & VMap, bool bBoundaryOnly = false);

	static bool CollapseTipFaces( VFTriangleMesh & mesh, bool bAll = true );
	static bool CollapseFinFaces( VFTriangleMesh & mesh, bool bAll = true );

	//! edge length will be multiplied by average of importance values for edge endpoints
	static bool MergeVertices( VFTriangleMesh & mesh, float fThreshold, const std::vector<float> * pImportanceMap = NULL, rms::MeshSelection * pSelection = NULL );

	//! collapse shortest edge of sliver triangles with an interior angle < fDegreeThreshold
	static bool CollapseSlivers( VFTriangleMesh & mesh, float fDegreeThreshold );


	static bool CheckForFins( VFTriangleMesh & mesh );
	static bool CheckMeshValidity( VFTriangleMesh & mesh, bool bTopologyChecks );

	//! bClosed flag is only set if bOrdered is true
	static bool VertexOneRing( VFTriangleMesh & mesh, IMesh::VertexID vID, std::vector<IMesh::VertexID> & vOneRing, bool bOrdered = false, bool * bClosed = NULL );

	//! bClosed flag is only set if bOrdered is true
	static bool TriangleOneRing( VFTriangleMesh & mesh, IMesh::VertexID vID, std::vector<IMesh::TriangleID> & vOneRing, bool bOrdered = false, bool * bClosed = NULL );


	static bool GetTriVerts( IMesh::VertexID nTri[3], IMesh::VertexID vID, IMesh::VertexID & other1, IMesh::VertexID & other2 );

	static void CotangentWeights( VFTriangleMesh & mesh, IMesh::VertexID vID, std::vector<IMesh::VertexID> & vOneRing, std::vector<float> & vWeights, bool bNormalize = false );
	static void UniformWeights( VFTriangleMesh & mesh, IMesh::VertexID vID, std::vector<IMesh::VertexID> & vOneRing, std::vector<float> & vWeights, bool bNormalize = false );

	static float VertexArea_Mixed( VFTriangleMesh & mesh, IMesh::VertexID vID );

	static Wml::Vector3f MeshLaplacian( VFTriangleMesh & mesh, IMesh::VertexID vID, std::vector<IMesh::VertexID> & vOneRing, std::vector<float> & vWeights );


	static void FindBoundaryLoops( const VFTriangleMesh & mesh, std::vector< std::vector< IMesh::VertexID > > & vLoops );
	static void FindBoundaryEdges( VFTriangleMesh & mesh, std::vector< std::pair<IMesh::VertexID,IMesh::VertexID> > & vEdges);

	struct BoundaryLoopMap {
		unsigned int nLoop1Index;
		unsigned int nLoop2Index;
		std::vector<IMesh::VertexID> vFrom;
		std::vector<IMesh::VertexID> vTo;
	};

	//! find corresponding boundary loops in two meshes. If mesh2 = mesh1, will intelligently skip repeats
	static void FindBoundaryLoopMaps(
		IMesh & mesh1, const std::vector< std::vector< IMesh::VertexID > > & vLoops1,
		IMesh & mesh2, const std::vector< std::vector< IMesh::VertexID > > & vLoops2,
		std::vector< BoundaryLoopMap > & vMaps, float fDistThresh = 0.000001f );

	static void MergeBoundaryLoops( VFTriangleMesh & mesh,
		const std::vector<IMesh::VertexID> & vFrom, const std::vector<IMesh::VertexID> & vTo );

	//! pass non-null vToLoops to have returned the VIDs of loop verts that were kept
	static void MergeAllBoundaryLoops( VFTriangleMesh & mesh, 
		std::vector< std::vector<IMesh::VertexID> > * vToLoops = NULL,
		float fDistThresh = 0.000001f  );

	static void CopyBoundaryUVs( VFTriangleMesh & from, VFTriangleMesh & to, IMesh::UVSetID nFromSetID, IMesh::UVSetID nToSetID,
		BoundaryLoopMap & bmap );

	static void GetBoundaryBoundingBox( VFTriangleMesh & mesh, Wml::AxisAlignedBox3f & bounds );
	static void GetBoundaryUVBoundingBox( VFTriangleMesh & mesh, Wml::AxisAlignedBox2f & bounds, IMesh::UVSetID nSetID = 0 );


	static void GetBoundaryEdgeStats( VFTriangleMesh & mesh, float & fMin, float & fMax, float & fAvg );
	static void GetEdgeLengthStats( IMesh * pMesh, float & fMin, float & fMax, float & fAvg );
	static void GetEdgeLengthStats( VFTriangleMesh & mesh, MeshSelection & selection, float & fMin, float & fMax, float & fAvg );

	// currently only does one-ring...
	static Wml::Vector3f GetAverageNormal( VFTriangleMesh & mesh, IMesh::VertexID vID, unsigned int nKRings = 1 );
	static void SmoothNormals( VFTriangleMesh & mesh );

	enum NormalEstMode {
		UniformFaceAvg,
		AreaWeightedFaceAvg
	};
	static Wml::Vector3f EstimateNormal( VFTriangleMesh & mesh, IMesh::VertexID vID, NormalEstMode eMode = AreaWeightedFaceAvg );
	static void EstimateNormals( VFTriangleMesh & mesh, NormalEstMode eMode = AreaWeightedFaceAvg, bool bSkipBoundary = false, Wml::Vector3f * pBuffer = NULL );

	static Wml::Vector3f FaceNormal( VFTriangleMesh & mesh, IMesh::TriangleID tID );
	static Wml::Vector3f FaceInterpNormal( IMesh * pMesh, IMesh::TriangleID tID, const Wml::Vector3f & vTriPoint );

	static Wml::Vector3f Centroid( const VFTriangleMesh & mesh );

	//! note: scale is currently not applied to normals...
	static void ScaleMesh( VFTriangleMesh & mesh, const Wml::Vector3f & vScale, const Wml::Vector3f & vCenter = Wml::Vector3f::ZERO );
	static void TranslateMesh( VFTriangleMesh & mesh, const Wml::Vector3f & vTranslate );
	static void RotateMesh( VFTriangleMesh & mesh, const Wml::Matrix3f & mRotate, const Wml::Vector3f & vCenter = Wml::Vector3f::ZERO );

	static void InvertMesh( VFTriangleMesh & mesh );

	static float LoopLength( const VFTriangleMesh & mesh, const std::vector<IMesh::VertexID> & vLoop );
	static float LoopLengthUV( const VFTriangleMesh & mesh, const std::vector<IMesh::VertexID> & vLoop, IMesh::UVSetID nSetID );
	static void ScaleMeshByLoop( VFTriangleMesh & mesh, const std::vector<IMesh::VertexID> & vLoop, float fTargetPerimeter );

	static float MeshArea( VFTriangleMesh & mesh );
	static float MeshUVArea( VFTriangleMesh & mesh, IMesh::UVSetID nUVSet = 0 );


	static void ScaleMeshUV( VFTriangleMesh & mesh, IMesh::UVSetID nSetID, const Wml::Vector2f & vScale, const Wml::Vector2f & vCenter = Wml::Vector2f::ZERO );

	static IMesh::VertexID FindGeodesicCenter( VFTriangleMesh & mesh, float * pMaxDistance = NULL );


	//! appends faces of vID to vSelection
	static void SelectFaces( VFTriangleMesh & mesh, IMesh::VertexID & vID, std::set<IMesh::TriangleID> & vSelection );

	//! appends all faces connected to vVerts to vSelection
	static void SelectFaces( VFTriangleMesh & mesh, const std::vector<IMesh::VertexID> & vVerts, std::set<IMesh::TriangleID> & vSelection );

	// vComponents is vector of size mesh->GetMaxTriangleID() where value is -1 if tID is not in vTris, otherwise component number < nComponents
	static void FindConnectedComponents(IMesh * pMesh, const std::set<IMesh::TriangleID> & vTris, std::vector<int> & vComponents, int & nComponents, int & nLargest);

	// TODO: this would be more efficient if we could find the boundary verts of a selection...
	//! appends all one-ring neighbours of current selection. pNewVerts returns added verts, if requested
	static void GrowSelection( VFTriangleMesh & mesh, 
							   std::set<IMesh::TriangleID> & vSelection, 
							   std::set<IMesh::VertexID> * pNewVerts );

	static void DrawBoundaryEdges( VFTriangleMesh & mesh, const Wml::Vector3f & vColor = Wml::Vector3f::UNIT_Z );
	static void DrawFrontEdges( VFTriangleMesh & mesh );


	//! returns the two vertices from nTri which != vOpp  (returns false if nTri does not contain vOpp)
	static bool PickTriVerts( IMesh::VertexID nTri[3], IMesh::VertexID vOpp, IMesh::VertexID & vOther1, IMesh::VertexID & vOther2 );


private:
	MeshUtils(void);
	~MeshUtils(void);
};


/*
 * This one-ring neighbour iteration constructs a list [parents] of all
 * the neighbouring vertices of [vertid] which are in [knownverts]
 */
class MakeOneRingListCallback : public IMesh::NeighborTriCallback
{
public:
	MakeOneRingListCallback( IMesh::VertexID vertid, VFTriangleMesh * mesh) {
		vID = vertid;
		pMesh = mesh;
	}
	IMesh::VertexID vID;
	std::set<IMesh::VertexID> vOneRing;
	VFTriangleMesh * pMesh;

	virtual void NextTriangle( IMesh::TriangleID tID ) {
		IMesh::VertexID nTri[3];
		pMesh->GetTriangle(tID, nTri);
		for ( int k = 0; k < 3; ++k ) {
			if ( nTri[k] != vID )
				vOneRing.insert( nTri[k] );
		}
	}
};





class MeshOneRingSource : public INeighbourSource<IMesh::VertexID>
{
public:
	MeshOneRingSource() { m_pMesh = NULL; }
	MeshOneRingSource(VFTriangleMesh * pMesh, bool bOrdered) { m_pMesh = pMesh; m_bOrderedOneRings = bOrdered; } 

	virtual void GetNeighbours( IMesh::VertexID nID, std::vector<IMesh::VertexID> & vNbrIDs ) {
		bool bOK = MeshUtils::VertexOneRing(*m_pMesh, nID, vNbrIDs, m_bOrderedOneRings );
		lgASSERT(bOK);
	}

	virtual IMesh::VertexID MaxID() { return m_pMesh->GetMaxVertexID(); }

	virtual bool IsOrdered() { return m_bOrderedOneRings; }

protected:
	VFTriangleMesh * m_pMesh;
	bool m_bOrderedOneRings;
};



class MeshPositionSource  : public IPositionSource<IMesh::VertexID>
{
public:
	MeshPositionSource() { m_pMesh = NULL; }
	MeshPositionSource(VFTriangleMesh * pMesh) { m_pMesh = pMesh; }

	virtual void GetPosition( IMesh::VertexID nID, Wml::Vector3f & vPosition, Wml::Vector3f * pNormal ) 
		{ m_pMesh->GetVertex(nID, vPosition, pNormal ); }
	virtual IMesh::VertexID MaxID() 
		{ return m_pMesh->GetMaxVertexID(); }

protected:
	VFTriangleMesh * m_pMesh;
};



class MeshGeoNbrSource : public INeighbourSource<IMesh::VertexID>
{
public:
	MeshGeoNbrSource() { m_pMesh = NULL; }
	MeshGeoNbrSource(VFTriangleMesh * pMesh, NeighbourCache<IMesh::VertexID> * pOneRingCache, MeshPositionSource * pPositionSource) 
		{ m_pMesh = pMesh; m_pOneRingCache = pOneRingCache; m_pPositionSource = pPositionSource; } 

	virtual void GetNeighbours( IMesh::VertexID nID, std::vector<IMesh::VertexID> & vNbrIDs ) {
		std::set<IMesh::VertexID> vNbrs;

		// hack - use two ring...
		const std::vector<IMesh::VertexID> & vOneRing = m_pOneRingCache->GetNeighbours(nID).vNbrs;
		size_t nOneRing = vOneRing.size();
		for ( unsigned int i = 0; i < nOneRing; ++i ) {
			vNbrs.insert( vOneRing[i] );
			const std::vector<IMesh::VertexID> & vTwoRing = m_pOneRingCache->GetNeighbours(vOneRing[i]).vNbrs;
			size_t nTwoRing = vTwoRing.size();
			for ( unsigned int k = 0; k < nTwoRing; ++k ) {
				if ( vTwoRing[k] != nID )
					vNbrs.insert( vTwoRing[k] );
			}
		}

		vNbrIDs = std::vector<IMesh::VertexID>( vNbrs.begin(), vNbrs.end() );
	}

	virtual IMesh::VertexID MaxID() { return m_pMesh->GetMaxVertexID(); }

	virtual bool IsOrdered() { return false; }

protected:
	VFTriangleMesh * m_pMesh;
	NeighbourCache<IMesh::VertexID> * m_pOneRingCache;
	MeshPositionSource * m_pPositionSource;
	
};



/*
 * simple classes for one-ring triangle iterations
 */

class NeighborTriPassThruCallback : public IMesh::NeighborTriCallback
{
public:
	NeighborTriPassThruCallback( IMesh::NeighborTriCallback * pPassThru )
		{ m_pPassThru = pPassThru; }

	virtual void NextTriangle( IMesh::TriangleID tID )
		{ m_pPassThru->NextTriangle(tID); }

protected:
	IMesh::NeighborTriCallback * m_pPassThru;
};


class NeighborTriBuffer : public IMesh::NeighborTriCallback
{
public:
	NeighborTriBuffer() {}

	const std::vector<IMesh::TriangleID> & Triangles() { return m_vTriangles; }

	virtual void BeginTriangles() 
		{ m_vTriangles.resize(0); }
	virtual void NextTriangle( IMesh::TriangleID tID )
		{ m_vTriangles.push_back(tID); }

protected:
	std::vector<IMesh::TriangleID> m_vTriangles;
};







}  // end namespace rms

