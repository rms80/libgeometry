// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#ifndef __RMS_MESH_BVTREE_H__
#define __RMS_MESH_BVTREE_H__

#include "config.h"
#include <Wm4AxisAlignedBox3.h>
#include <Wm4Ray3.h>

#include "IMesh.h"
#include "MemoryPool.h"



namespace rms {

class IMeshBVTree
{
public:
	IMeshBVTree( );
	IMeshBVTree( IMesh * pMesh );

	void SetMesh( IMesh * pMesh );

	void Clear();

	void GetMeshBounds( Wml::AxisAlignedBox3f & bounds );

	bool FindRayIntersection( const Wml::Vector3f & vOrigin, const Wml::Vector3f & vDirection,
							  Wml::Vector3f & vHit, IMesh::TriangleID & nHitTri );

	bool FindNearest( const Wml::Vector3f & vPoint, Wml::Vector3f & vNearest, IMesh::TriangleID & nNearestTri );

	bool FindNearestVtx( const Wml::Vector3f & vPoint, IMesh::VertexID & nNearestVtx );

	//! get nearest distance for last query
	float LastDistance() { return m_fLastQueryDistance; }

	//! expand entire BV Tree (makes queries faster, but expensive)
	void ExpandAll();

protected:
	IMesh * m_pMesh;

	float m_fLastQueryDistance;

	class IMeshBVNode {
	public:
		unsigned int ID;
		Wml::AxisAlignedBox3f Box;
		IMeshBVTree::IMeshBVNode * pLeft;
		IMeshBVTree::IMeshBVNode * pRight;

		union {
			struct {
				unsigned int Index : 31;
				unsigned int NotLeaf : 1;
			} TriangleList;
			unsigned int ID;
		} Triangle;

		inline bool HasChildren() { 
				return pLeft != NULL && pRight != NULL; }

		inline bool IsLeaf() {
				return Triangle.TriangleList.NotLeaf == 0; }

		inline void SetIndex( unsigned int nIndex ) {
				Triangle.TriangleList.NotLeaf = 1;
				Triangle.TriangleList.Index = nIndex; }
		inline unsigned int GetIndex() { 
				return Triangle.TriangleList.Index; }

		inline void SetTriangleID( unsigned int nID ) {
				Triangle.TriangleList.NotLeaf = 0;
				Triangle.ID = nID; }
		inline unsigned int GetTriangleID() {
				return Triangle.ID; }

		inline void Union( Wml::Vector3f & vPoint );
	};


	
	MemoryPool<IMeshBVNode> m_vNodePool;
	IMeshBVNode * m_pRoot;
	unsigned int m_nNodeIDGen;
	IMeshBVNode * GetNewNode();

	void Initialize();


	struct TriangleEntry {
		unsigned int nJump : 12;		// 0xFFF
		unsigned int nNodeID  : 20;		
		IMesh::TriangleID triID;
	};
	std::vector<TriangleEntry> m_vTriangles;
	unsigned int m_nMaxTriangle;

	inline unsigned int GetNextEntryIdx( IMeshBVNode * pNode, unsigned int nIndex );
	inline void SetJump( unsigned int nIndex, unsigned int nNext );


	struct Ray {
		Ray( const Wml::Vector3f & vOrigin, const Wml::Vector3f & vDirection );
		Wml::Ray3f wmlRay;
		Wml::Vector3f origin;
		Wml::Vector3f direction;
		Wml::Vector3f inv_direction;
		int sign[3];
	};

	void ComputeBox( IMeshBVNode * pNode );
	void ExpandNode( IMeshBVNode * pNode );
	bool TestIntersection( IMeshBVNode * pNode, Ray & ray, float & fNear, float & fFar );
	float MinDistance( IMeshBVNode * pNode, const Wml::Vector3f & vPoint );

	//! recursive intersection test
	bool FindRayIntersection( IMeshBVTree::IMeshBVNode * pNode, Ray & ray, 
							  Wml::Vector3f & vHit, float & fNearest, IMesh::TriangleID & nHitTri );

	bool FindNearest( IMeshBVTree::IMeshBVNode * pNode, const Wml::Vector3f & vPoint, 
					  Wml::Vector3f & vNearest, float & fNearest, IMesh::TriangleID & nNearestTri );

	void ExpandAll( IMeshBVTree::IMeshBVNode * pNode );
};



void IMeshBVTree::IMeshBVNode::Union( Wml::Vector3f & point )
{
	for ( int k = 0; k < 3; ++k ) {
		if ( point[k] < Box.Min[k] )
			Box.Min[k] = point[k];
		if ( point[k] > Box.Max[k] )
			Box.Max[k] = point[k];
	}
}


unsigned int IMeshBVTree::GetNextEntryIdx( IMeshBVTree::IMeshBVNode * pNode, unsigned int nIndex )
{
	nIndex = nIndex + m_vTriangles[nIndex].nJump;
	while ( nIndex < m_nMaxTriangle ) {
		if ( m_vTriangles[nIndex].nNodeID == pNode->ID )
			return nIndex;
		else
			++nIndex;	// linear iteration to next entry...
	}
	return IMesh::InvalidID;
}

void IMeshBVTree::SetJump( unsigned int nIndex, unsigned int nNext )
{
	if ( nNext - nIndex < 0xFFF ) {
		m_vTriangles[nIndex].nJump = (nNext - nIndex) & 0xFFF;
	} else {
		m_vTriangles[nIndex].nJump = 0xFFF;
	}
	if(m_vTriangles[nIndex].nJump == 0)
		lgBreakToDebugger();
}


}  // namespace rmsmesh

#endif // __RMS_MESH_BVTREE_H__