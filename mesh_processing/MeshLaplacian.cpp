// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#include "MeshLaplacian.h"
#include "VectorUtil.h"
#include "MeshUtils.h"
#include "VertexWeights.h"

#include "rmsdebug.h"

using namespace rms;



MeshLaplacian::MeshLaplacian( VFTriangleMesh * pMesh, WeightType eWeightType ) 
{
	m_pMesh = pMesh;
	m_eWeightType = eWeightType;

	m_vVertices.resize( m_pMesh->GetMaxVertexID() );
}


MeshLaplacian::VertexLaplacian & MeshLaplacian::Get(IMesh::VertexID vID, unsigned int nOrder)
{
	VertexSet & v = m_vVertices[vID];
	while ( v.vOrders.size() < nOrder )
		ComputeLaplacian( vID, (unsigned int)v.vOrders.size()+1 );
	return v.vOrders[nOrder-1];
}

const MeshLaplacian::VertexLaplacian & MeshLaplacian::Get(IMesh::VertexID vID, unsigned int nOrder) const
{
	return const_cast<MeshLaplacian *>(this)->Get(vID, nOrder);
}


void MeshLaplacian::ComputeLaplacian( IMesh::VertexID vID, unsigned int nOrder )
{
	VertexSet & v = m_vVertices[vID];
	if ( v.vOrders.size() < nOrder )
		v.vOrders.resize(nOrder);
	VertexLaplacian & l = v.vOrders[nOrder-1];
	if ( nOrder == 1 ) {
		l.nOrder = 1;
		m_pMesh->VertexOneRing(vID, l.vNbrs);

		l.fWi = 1.0f;

		MeshUtils::CotangentWeights(*m_pMesh, vID, l.vNbrs, l.vWij);

		l.fWii = 0;
		size_t nNbrs = l.vNbrs.size();
		for ( unsigned int k = 0; k < nNbrs; ++k )
			l.fWii -= l.vWij[k];

	} else {

		std::map<IMesh::VertexID, float> vNbrs;

		VertexLaplacian & l1 = Get(vID, 1);
		size_t nOneRing = l1.vNbrs.size();

		VertexLaplacian & li = Get(vID, nOrder-1);
		for ( unsigned int k = 0; k < nOneRing; ++k ) {
			IMesh::VertexID vNbrID = l1.vNbrs[k];
			VertexLaplacian & lj = Get( vNbrID, nOrder-1 );

			float wij = l1.vWij[k];
			
			size_t nNbrsj = lj.vNbrs.size();
			for ( unsigned int ji = 0; ji < nNbrsj; ++ji ) {
				IMesh::VertexID jiID = lj.vNbrs[ji];
				float wjj = lj.vWij[ji] * lj.fWi;
				vNbrs[jiID] += wij*wjj;
			}
			vNbrs[vNbrID] += wij * lj.fWi * lj.fWii;

			size_t nNbrsi = li.vNbrs.size();
			for ( unsigned int ii = 0; ii < nNbrsi; ++ii ) {
				IMesh::VertexID iiID = li.vNbrs[ii];
				float wii = li.vWij[ii] * li.fWi;
				vNbrs[iiID] += wij * wii;
			}
			vNbrs[vID] += wij * li.fWi * li.fWii;
		}

		l.fWi = 1.0f;

		std::map<IMesh::VertexID, float>::iterator curv(vNbrs.begin()), endv(vNbrs.end());
		while ( curv != endv ) {
			IMesh::VertexID id = curv->first;
			float w = curv->second;
			++curv;
		
			if ( id == vID ) {
				l.fWii = w;
			} else {
				l.vNbrs.push_back(id);
				l.vWij.push_back(w);
			}
		}
	}
		

}
