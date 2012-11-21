// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#include <VFTriangleMesh.h>
#include "DijkstraFrontProp.h"


namespace rms {


class FacePositionSource : public IPositionSource<IMesh::TriangleID>
{
public:
	std::vector<Wml::Vector3f> * pCenters;
	FacePositionSource(std::vector<Wml::Vector3f> * centers)
		{ pCenters = centers; }

	virtual void GetPosition( IMesh::TriangleID nID, Wml::Vector3f & vPosition, Wml::Vector3f * pNormal ) {
		vPosition = (*pCenters)[nID];
	}
	virtual IMesh::TriangleID MaxID() {
		return (IMesh::TriangleID)pCenters->size();
	}
};

class TriangleNeighbourSource : public INeighbourSource<IMesh::TriangleID>
{
public:
	VFTriangleMesh * pMesh;
	TriangleNeighbourSource( VFTriangleMesh * mesh ) 
		{ pMesh = mesh; }

	virtual void GetNeighbours( IMesh::TriangleID nID, std::vector<IMesh::TriangleID> & vNbrIDs )
	{ IMesh::TriangleID tNbrs[3];
	  pMesh->FindNeighbours(nID, tNbrs);
	  for ( int j = 0; j < 3; ++j )
		  if ( tNbrs[j] != IMesh::InvalidID )
			  vNbrIDs.push_back(tNbrs[j]);
	}
	virtual IMesh::TriangleID MaxID() 
		{ return pMesh->GetMaxTriangleID(); }
};




class VertexPositionSource : public IPositionSource<IMesh::TriangleID>
{
public:
	VFTriangleMesh * pMesh;
	VertexPositionSource(VFTriangleMesh * mesh)
		{ pMesh = mesh; }

	virtual void GetPosition( IMesh::VertexID nID, Wml::Vector3f & vPosition, Wml::Vector3f * pNormal ) {
		pMesh->GetVertex(nID, vPosition, pNormal);
	}
	virtual IMesh::VertexID MaxID() {
		return pMesh->GetMaxVertexID();
	}
};

class VertexOneRingNeighbourSource : public INeighbourSource<IMesh::TriangleID>
{
public:
	VFTriangleMesh * pMesh;
	VertexOneRingNeighbourSource( VFTriangleMesh * mesh ) 
		{ pMesh = mesh; }

	virtual void GetNeighbours( IMesh::VertexID nID, std::vector<IMesh::VertexID> & vNbrIDs ) { 
		pMesh->VertexOneRing(nID, vNbrIDs, false);
	}
	virtual IMesh::VertexID MaxID() {
		return pMesh->GetMaxVertexID();
	}
};


}  // end namespace rms