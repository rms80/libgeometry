// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#include "MeshCurvature.h"
#include "VectorUtil.h"
#include "MeshUtils.h"
#include "Wm4GMatrix.h"
#include "Wm4LinearSystem.h"

using namespace rms;


MeshCurvature::MeshCurvature()
{
}




void MeshCurvature::MeanCurvature( MeanCurvatureMode eMode, VFTriangleMesh * pMesh, const std::vector<IMesh::VertexID> & vSelection )
{
	if ( m_vH.size() != pMesh->GetMaxVertexID() ) {
		m_vH.resize(0);
		m_vH.resize(pMesh->GetMaxVertexID(), 0);
	}

	if ( ! vSelection.empty() ) {
		size_t nCount = vSelection.size();
		for ( unsigned int k = 0; k < nCount; ++k )
			MeanCurvature( eMode, pMesh, vSelection[k] );
	} else {
		VFTriangleMesh::vertex_iterator curv(pMesh->BeginVertices()), endv(pMesh->EndVertices());
		while ( curv != endv ) {
			IMesh::VertexID vID = *curv;  ++curv;
			MeanCurvature( eMode, pMesh, vID );
		}
	}
}

float MeshCurvature::MeanCurvature( MeanCurvatureMode eMode, VFTriangleMesh * pMesh, IMesh::VertexID vID)
{
	if ( m_vH.size() != pMesh->GetMaxVertexID() ) {
		m_vH.resize(0);
		m_vH.resize(pMesh->GetMaxVertexID(), 0);
	}

	switch ( eMode ) {
		default:
		case MeanCurvature_Normal:
			m_vH[vID] = MeanCurvature_NormalSK01(pMesh, vID);
	}

	return m_vH[vID];
}






void tangentFrame( const Wml::Vector3f & n, Wml::Vector3f & tan1, Wml::Vector3f & tan2 )
{
	if ( fabs(n[0]) >= fabs(n[1]) && fabs(n[0]) >= fabs(n[2]) )
		tan1 = Wml::Vector3f( -n[1], n[0], 0 );
	else
		tan1 = Wml::Vector3f( 0, n[2], n[1] );
	tan1.Normalize();
	tan2 = tan1.Cross(n);
}


// CIRCLE2_2PR find circle centers from 2 points and radius (2 results)
//   based on http://mathforum.org/library/drmath/view/53027.html
void circle2_2pr( const Wml::Vector2f & p1, const Wml::Vector2f & p2, float r, Wml::Vector2f & c1, Wml::Vector2f & c2 ) 
{
	float q = sqrt( (p2-p1).SquaredLength() );
	Wml::Vector2f h = (p1+p2)/2;
	Wml::Vector2f m( p1[1]-p2[1], p2[0]-p1[0] );
	float d = sqrt(r*r - (q/2)*(q/2)) / q;
	c1 = h + (d*m);
	c2 = h - (d*m);
}



//ISECT_LINE2_CIRCLE2 compute intersection points between line and circle
//   based on http://www.geometrictools.com/Documentation/IntersectionLine2Circle2.pd
bool isect_line2_circle2( const Wml::Vector2f & p0, const Wml::Vector2f & p1, const Wml::Vector2f & c, float r, Wml::Vector2f vHits[2] )
{
	Wml::Vector2f d( p1-p0 );
	d.Normalize();
	Wml::Vector2f k( p0 - c );

	float dkDot = d.Dot(k);
	float discrim = dkDot*dkDot - d.SquaredLength()*( k.SquaredLength() - r*r);
	if (discrim < 0) {
		return false;
	} else {
		float t0 = (-dkDot + sqrt(discrim)) / d.SquaredLength();
		float t1 = (-dkDot - sqrt(discrim)) / d.SquaredLength();
		vHits[0] = p0 + t0*d;
		vHits[1] = p0 + t1*d;
		return true;
	}
}




float MeshCurvature::MeanCurvature_NormalSK01( VFTriangleMesh * pMesh, IMesh::VertexID vID )
{
	std::vector<IMesh::VertexID> vOneRing;
	MeshUtils::VertexOneRing( *pMesh, vID, vOneRing, false);
	size_t nOneRing = vOneRing.size();

	Wml::Vector3f qi, ni;
	pMesh->GetVertex(vID, qi, &ni);

	std::vector<Wml::Vector3f> Qj(nOneRing);
	for ( unsigned int i = 0; i < nOneRing; ++i ) {
		pMesh->GetVertex( vOneRing[i], Qj[i] );
		Qj[i] -= qi;
	}

	if ( nOneRing < 5 ) {

		// estimate new points and append to Qj
		std::vector<IMesh::VertexID> vOneRingT;
		MeshUtils::TriangleOneRing(*pMesh, vID, vOneRingT, false);
		size_t nTris = vOneRingT.size();
		for ( unsigned int k = 0; k < nTris; ++k ) {

			// figure out which nbr verts to use
			IMesh::VertexID nTri[3];
			pMesh->GetTriangle( vOneRingT[k], nTri );
			IMesh::VertexID q1, q2;
			MeshUtils::GetTriVerts( nTri, vID, q1, q2 );

			// find faces for that edge  [TODO] utility function for this
			IMesh::EdgeID e1 = pMesh->FindEdge(q1,q2);
			IMesh::VertexID nEdgeV[2];  IMesh::TriangleID nEdgeT[2];
			pMesh->GetEdge(e1, nEdgeV, nEdgeT);
			if ( nEdgeT[0] == IMesh::InvalidID || nEdgeT[1] == IMesh::InvalidID )
				continue;		// edge triangle
			IMesh::TriangleID f1 = nEdgeT[0];
			IMesh::TriangleID f2 = nEdgeT[1];

			// construct E plane
			Wml::Vector3f v1,v2, n1, n2;
			pMesh->GetVertex(q1,v1,&n1);
			pMesh->GetVertex(q2,v2,&n2);
			Wml::Vector3f Et1( Normalize(v2-v1) );
			Wml::Vector3f Et2( MeshUtils::FaceNormal(*pMesh,f1) + MeshUtils::FaceNormal(*pMesh,f2) );   Et2.Normalize();
			Wml::Vector3f En( NCross(Et1,Et2) );

			// construct tangent vectors
			Wml::Vector3f t1( n1.Cross(En) );   t1.Normalize();
			if ( VectorAngle(-t1, Et1) < VectorAngle(t1,Et1) )
				t1 = -t1;
			Wml::Vector3f t2( n2.Cross(En) );   t2.Normalize();
			if ( VectorAngle(-t2, Et1) < VectorAngle(t2,Et1) )
				t2 = -t2;

			// construct basis for E plane and project points into it
			Wml::Vector3f bx,by;
			tangentFrame(En, bx, by);
			Wml::Vector3f Eo = v1;
			Wml::Vector2f V1( (v1-Eo).Dot(bx), (v1-Eo).Dot(by) );
			Wml::Vector2f V2( (v2-Eo).Dot(bx), (v2-Eo).Dot(by) );
			Wml::Vector2f T1( t1.Dot(bx), t1.Dot(by) );   T1.Normalize();
			Wml::Vector2f T2( t2.Dot(bx), t2.Dot(by) );   T2.Normalize();

			// compute circles
			float r1 = 2*(V2-V1).SquaredLength() / (V2-V1).Dot( T1.Perp() );
			Wml::Vector2f ca,cb;
			circle2_2pr(V1, V2, r1, ca,cb);
			Wml::Vector2f tmp = Normalize(V1-ca);
			Wml::Vector2f c1 = ( tmp.Dot(T1) > 0.1f ) ? cb : ca;
			float r2 = 2*(V2-V1).SquaredLength() / (V2-V1).Dot( T2.Perp() );
			circle2_2pr(V1, V2, r2, ca,cb);
			tmp = Normalize(V2-ca);
			Wml::Vector2f c2 = ( tmp.Dot(T2) > 0.1f ) ? cb : ca;

			if ( ! _finite(r1) || ! _finite(r2) )
				continue;		// does this happen? why?

            // intersect perp bisector with circles
			// [NOTE] should actually be using nearest point on line (v1->v2), instead of nearest point to midpoint 0.5(v1+v2)
			Wml::Vector2f pO = 0.5f * (V1+V2);
			Wml::Vector2f pD = (V2-V1).Perp();
			Wml::Vector2f hits[2];
			if ( ! isect_line2_circle2(pO,pO+pD,c1,r1, hits) )
				continue;
			Wml::Vector2f hit1 = ( (hits[0]-pO).SquaredLength() < (hits[1]-pO).SquaredLength() ) ? hits[0] : hits[1];
			if ( ! isect_line2_circle2(pO,pO+pD,c2,r2, hits) )
				continue;
			Wml::Vector2f hit2 = ( (hits[0]-pO).SquaredLength() < (hits[1]-pO).SquaredLength() ) ? hits[0] : hits[1];

			// project new point back to 3D
			Wml::Vector2f p = 0.5f * ( hit1 + hit2 );
			Wml::Vector3f P = Eo + p[0]*bx + p[1]*by;
			Qj.push_back(P - qi);
		}

	}

	int nQ = (int)Qj.size();

	Wml::Vector3f bx, by;
	tangentFrame(ni, bx, by);

	std::vector<Wml::Vector2f> Tj(nQ);
	for ( int i = 0; i < nQ; ++i ) {
		Tj[i] = Wml::Vector2f( Qj[i].Dot(bx), Qj[i].Dot(by) );
		Tj[i].Normalize();
	}

	Wml::GMatrixf A(nQ, 3);
	for ( int i = 0; i < nQ; ++i ) {
		A(i,0) = Tj[i].X() * Tj[i].X();
		A(i,1) = Tj[i].X() * Tj[i].Y();
		A(i,2) = Tj[i].Y() * Tj[i].Y();
	}

	Wml::GVectorf K(nQ);
	for ( int i = 0; i < nQ; ++i )
		K[i] = 2 * Qj[i].Dot(-ni) / Qj[i].Dot(Qj[i]);


	// solve by normal equations
	Wml::GMatrixf AT = A.Transpose();
	Wml::GMatrixf ATA = AT * A;
	Wml::GVectorf ATK = AT * K;

	std::vector<float> X(nQ);
	Wml::LinearSystemf linsys;
	bool bOK = linsys.SolveSymmetricCG( ATA, ATK, &X[0] );
	if ( !bOK )
		lgBreakToDebugger();

	float H = (X[0] + X[2]) / 2.0f;
	return H;
}




