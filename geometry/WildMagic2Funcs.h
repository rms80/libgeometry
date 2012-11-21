// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#pragma once


/*
 * [RMS] ports of intersection/distance functions from WildMagic2
 */

#include <Wm4Segment2.h>

#include <Wm4IntrRay3Triangle3.h>
#include <Wm4IntrRay3Sphere3.h>
#include <Wm4IntrLine3Sphere3.h>
#include <Wm4DistVector3Line3.h>
#include <Wm4DistVector3Circle3.h>
#include <Wm4DistLine3Ray3.h>
#include <Wm4DistLine3Circle3.h>


namespace Wml {


/*
 * Vector/Line distance
 */
template <class Real>
static Real SqrDistance( const Wml::Vector3<Real> & vec, const Wml::Line3<Real> & line, Real * pLineParam = NULL )
{
	Wml::DistVector3Line3<Real> dist(vec, line);
	Real fSqrDist = dist.GetSquared();
	if ( pLineParam )
		*pLineParam = dist.GetLineParameter();
	return fSqrDist;
}
/*
 * Vector/Circle distance
 */
template <class Real>
static Real SqrDistance( const Wml::Vector3<Real> & vec, const Wml::Circle3<Real> & circle, Wml::Vector3<Real> * pkCircleClosest = NULL )
{
	Wml::DistVector3Circle3<Real> dist(vec, circle);
	Real fSqrDist = dist.GetSquared();
	if ( pkCircleClosest )
		*pkCircleClosest = dist.GetClosestPoint1();
	return fSqrDist;
}



/*
 * line/ray distance
 */
template <class Real>
static Real SqrDistance( const Wml::Line3<Real> & line, const Wml::Ray3<Real> & ray, Real * pLineCoeff = NULL, Real * pRayCoeff = NULL )
{
	Wml::DistLine3Ray3<Real> dist(line,ray);
	Real fDistSqr = dist.GetSquared();
	if ( pLineCoeff )
		*pLineCoeff = dist.GetLineParameter();
	if ( pRayCoeff )
		*pRayCoeff = dist.GetRayParameter();
	return fDistSqr;
}


/*
 * Line/Circle distance
 */
template <class Real>
static Real SqrDistance( const Wml::Line3<Real> & line, const Wml::Circle3<Real> & circle, Wml::Vector3<Real> * pkLineClosest = NULL, Wml::Vector3<Real> * pkCircleClosest = NULL )
{
	Wml::DistLine3Circle3<Real> dist(line,circle);
	Real fSqrDist = dist.GetSquared();
	if ( pkLineClosest )
		*pkLineClosest = dist.GetClosestPoint0();
	if ( pkCircleClosest )
		*pkCircleClosest = dist.GetClosestPoint1();
	return fSqrDist;
}



/*
 * Line/Sphere distance
 */
template <class Real>
static Real Wml2Distance(const Wml::Line3<Real> & rkLine, const Wml::Sphere3<Real> & rkSphere, Wml::Vector3<Real> * pkLineClosest = NULL, Wml::Vector3<Real> * pkSphereClosest = NULL )
{
	Real fLineParam;
	Real rCentreDist = Wml::SqrDistance( rkSphere.Center, rkLine, &fLineParam);
	rCentreDist = (Real)sqrt(rCentreDist);

	if (rCentreDist < 0) {
		if (pkLineClosest || pkSphereClosest) {
			Vector3<Real> vIntersections[2];
			int iQuantity;
			FindIntersection( rkLine, rkSphere, iQuantity, vIntersections );
			*pkLineClosest = *pkSphereClosest = vIntersections[0];
		}
		return 0.0;

	} else {
		if (pkSphereClosest) {
			Vector3<Real> ptLine( rkLine.Origin + rkLine.Direction * fLineParam );
			Vector3<Real> vDir( ptLine - rkSphere.Center );
			vDir.Normalize();
			*pkSphereClosest = rkSphere.Center + vDir * rkSphere.Radius;
			if (pkLineClosest) {
				*pkLineClosest = ptLine;
			}
		} else if (pkLineClosest) {
			*pkLineClosest = Vector3<Real>( rkLine.Origin + rkLine.Direction * fLineParam );
		}
		return 	rCentreDist - rkSphere.Radius;
	}
}







/*
 * Ray/Triangle intersection
 */
template <class Real>
static bool FindIntersection(const Wml::Ray3<Real> & ray, const Wml::Triangle3<Real> & tri, Wml::Vector3<Real> & vHit)
{
	Wml::IntrRay3Triangle3<Real> intr(ray, tri);
	if ( ! intr.Find() )
		return false;
	vHit = ray.Origin + intr.GetRayT() * ray.Direction;
	return true;
}
/*
 * Ray/Sphere intersection
 */
template <class Real>
static bool FindIntersection(const Wml::Ray3<Real> & ray, const Wml::Sphere3<Real> & sphere, int & iQuantity, Wml::Vector3<Real> pHits[2] )
{
	Wml::IntrRay3Sphere3<Real> intr(ray, sphere);
	if ( ! intr.Find() )
		return false;
	iQuantity = intr.GetQuantity();
	for ( int k = 0; k < iQuantity; ++k )
		pHits[k] = ray.Origin + intr.GetRayT(k) * ray.Direction;
	return true;
}
/*
 * Line/Sphere intersection
 */
template <class Real>
static bool FindIntersection(const Wml::Line3<Real> & line, const Wml::Sphere3<Real> & sphere, int & iQuantity, Wml::Vector3<Real> pHits[2] )
{
	Wml::IntrLine3Sphere3<Real> intr(line, sphere);
	if ( ! intr.Find() )
		return false;
	iQuantity = intr.GetQuantity();
	for ( int k = 0; k < iQuantity; ++k )
		pHits[k] = line.Origin + intr.GetLineT(k) * line.Direction;
	return true;
}




} // end namespace Wml



/*
 * Ports of WildMagic2 functions that seemed to have disappeared in WildMagic3+
 */

namespace Wml2 {



/*
 * 2D Segment/Segment Intersection
 */


// [RMS] port of WildMagic2 IntrLin2Lin2 code to find intersection between two line segments
//   ??? how should I be doing this in Wml4 ???
template <class Real>
static bool Find (const Wml::Vector2<Real>& rkP0, const Wml::Vector2<Real>& rkD0,  const Wml::Vector2<Real>& rkP1, const Wml::Vector2<Real>& rkD1,
				  Wml::Vector2<Real>& rkDiff, Real& rfD0SqrLen, int& riQuantity, Real afT[2])
{
    // Intersection is a solution to P0+s*D0 = P1+t*D1.  Rewrite as
    // s*D0 - t*D1 = P1 - P0, a 2x2 system of equations.  If D0 = (x0,y0)
    // and D1 = (x1,y1) and P1 - P0 = (c0,c1), then the system is
    // x0*s - x1*t = c0 and y0*s - y1*t = c1.  The error tests are relative
    // to the size of the direction vectors, |Cross(D0,D1)| >= e*|D0|*|D1|
    // rather than absolute tests |Cross(D0,D1)| >= e.  The quantities
    // P1-P0, |D0|^2, and |D1|^2 are returned for use by calling functions.

    Real fDet = rkD1.DotPerp(rkD0);
    rkDiff = rkP1 - rkP0;
    rfD0SqrLen = rkD0.SquaredLength();

    if ( fDet*fDet > Wml::Math<Real>::EPSILON*rfD0SqrLen*rkD1.SquaredLength() ) {
        // Lines intersect in a single point.  Return both s and t values for
        // use by calling functions.
        Real fInvDet = ((Real)1.0)/fDet;
        riQuantity = 1;
        afT[0] = rkD1.DotPerp(rkDiff)*fInvDet;
        afT[1] = rkD0.DotPerp(rkDiff)*fInvDet;
    } else {
        // lines are parallel
        fDet = rkD0.DotPerp(rkDiff);
        Real fRHS = Wml::Math<Real>::EPSILON*rfD0SqrLen*rkDiff.SquaredLength();
		riQuantity = ( fDet*fDet > fRHS ) ? 0 : 2;   // lines are disjoint / the same
    }

    return riQuantity != 0;
}
template <class Real>
bool FindIntersection (const Wml::Segment2<Real>& rkSegment0,
   const Wml::Segment2<Real>& rkSegment1, int& riQuantity, Real afT[2])
{
	Wml::Vector2<Real> kDiff;
    Real fD0SqrLen;
    bool bIntersects = Find(rkSegment0.Origin,rkSegment0.Direction,
        rkSegment1.Origin,rkSegment1.Direction,kDiff,fD0SqrLen,
        riQuantity,afT);

    if ( bIntersects ) {
        if ( riQuantity == 1 ) {
            if ( afT[0] < (Real)0.0 || afT[0] > (Real)1.0 ||   afT[1] < (Real)0.0 || afT[1] > (Real)1.0 )
                riQuantity = 0;		// lines intersect, but segments do not
        } else{
            // segments are on the same line
            Real fDotRS = rkSegment0.Direction.Dot(rkSegment1.Direction);
            Real fDot0, fDot1;
            if ( fDotRS > (Real)0.0 ) {
                fDot0 = kDiff.Dot(rkSegment0.Direction);
                fDot1 = fDot0 + fDotRS;
            } else {
                fDot1 = kDiff.Dot(rkSegment0.Direction);
                fDot0 = fDot1 + fDotRS;
            }

            // compute intersection of [t0,t1] and [0,1]
            if ( fDot1 < (Real)0.0 || fDot0 > fD0SqrLen ) {
                riQuantity = 0;
            } else if ( fDot1 > (Real)0.0 ) {
                if ( fDot0 < fD0SqrLen ) {
                    Real fInvLen = ((Real)1.0)/fD0SqrLen;
                    riQuantity = 2;
                    afT[0] = (fDot0 < (Real)0.0 ? (Real)0.0 : fDot0*fInvLen);
                    afT[1] = (fDot1 > fD0SqrLen ? (Real)1.0 : fDot1*fInvLen);
				}  else  { // fT0 == 1  
                    riQuantity = 1;
                    afT[0] = (Real)1.0;
                }
			} else {  // fT1 == 0
                riQuantity = 1;
                afT[0] = (Real)0.0;
            }
        }
    }

    return riQuantity != 0;
}




} // end namespace Wml