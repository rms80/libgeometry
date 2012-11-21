// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#ifndef __RMSMESH_MESH_UVBVTREE_H__
#define __RMSMESH_MESH_UVBVTREE_H__

#include "config.h"

#include <IMesh.h>
#include <Wm4AxisAlignedBox2.h>
#include <MemoryPool.h>
#include "rmsprofile.h"

namespace rms {

class  IMeshUVBVTree
{
public:
	IMeshUVBVTree( );
	IMeshUVBVTree( IMesh * pMesh );

	void SetMesh( IMesh * pMesh );

	void Clear();

	bool FindTriangle( const Wml::Vector2f & vUV, IMesh::TriangleID & nTri );
	bool FindNearestTriangle( const Wml::Vector2f & vUV, Wml::Vector2f & vNearest, IMesh::TriangleID & nTri );

	void ExpandAll();

protected:
	IMesh * m_pMesh;

	class IMeshUVBVNode {
	public:
		unsigned int ID;
		Wml::AxisAlignedBox2f Box;
		IMeshUVBVTree::IMeshUVBVNode * pLeft;
		IMeshUVBVTree::IMeshUVBVNode * pRight;

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

		inline void Union( float fX, float fY );
	};

	
	MemoryPool<IMeshUVBVNode> m_vNodePool;
	IMeshUVBVNode * m_pRoot;
	unsigned int m_nNodeIDGen;
	IMeshUVBVNode * GetNewNode();

	void Initialize();

	struct TriangleEntry {
		unsigned int nJump : 12;		// 0xFFF
		unsigned int nNodeID  : 20;		
		IMesh::TriangleID triID;
	};
	std::vector<TriangleEntry> m_vTriangles;
	unsigned int m_nMaxTriangle;

	inline unsigned int GetNextEntryIdx( IMeshUVBVNode * pNode, unsigned int nIndex );
	inline void SetJump( unsigned int nIndex, unsigned int nNext );


	void ComputeBox( IMeshUVBVNode * pNode );
	void ExpandNode( IMeshUVBVNode * pNode );
	void ExpandAll( IMeshUVBVNode * pNode );
	bool IsInside( IMeshUVBVNode * pNode, float fX, float fY );
	float MinDistance( IMeshUVBVNode * pNode, const Wml::Vector2f & vPoint );


	bool FindTriangle( IMeshUVBVTree::IMeshUVBVNode * pNode, const Wml::Vector2f & vUV, IMesh::TriangleID & nTri );
	bool FindNearestTriangle( IMeshUVBVTree::IMeshUVBVNode * pNode, const Wml::Vector2f & vUV, 
		Wml::Vector2f & vNearest, float & fNearest, IMesh::TriangleID & nTri );
};



void IMeshUVBVTree::IMeshUVBVNode::Union( float fX, float fY )
{
	if ( fX < Box.Min[0] )
		Box.Min[0] = fX;
	if ( fX > Box.Max[0] )
		Box.Max[0] = fX;
	if ( fY < Box.Min[1] )
		Box.Min[1] = fY;
	if ( fY > Box.Max[1] )
		Box.Max[1] = fY;
}


unsigned int IMeshUVBVTree::GetNextEntryIdx( IMeshUVBVTree::IMeshUVBVNode * pNode, unsigned int nIndex )
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

void IMeshUVBVTree::SetJump( unsigned int nIndex, unsigned int nNext )
{
	if ( nNext - nIndex < 0xFFF ) {
		m_vTriangles[nIndex].nJump = (nNext - nIndex) & 0xFFF;
	} else {
		m_vTriangles[nIndex].nJump = 0xFFF;
	}
	if ( m_vTriangles[nIndex].nJump == 0 )
		lgBreakToDebugger();
}


}  // namespace rmsmesh

#endif // __RMSMESH_MESH_UVBVTREE_H__
