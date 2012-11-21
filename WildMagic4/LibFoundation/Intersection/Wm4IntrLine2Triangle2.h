// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTRLINE2TRIANGLE2_H
#define WM4INTRLINE2TRIANGLE2_H

#include "Wm4FoundationLIB.h"
#include "Wm4Intersector.h"
#include "Wm4Line2.h"
#include "Wm4Triangle2.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntrLine2Triangle2
    : public Intersector<Real,Vector2<Real> >
{
public:
    IntrLine2Triangle2 (const Line2<Real>& rkLine,
        const Triangle2<Real>& rkTriangle);

    // Object access.
    const Line2<Real>& GetLine () const;
    const Triangle2<Real>& GetTriangle () const;

    // Static intersection query.
    virtual bool Test ();
    virtual bool Find ();

    // The intersection set.  If the line and triangle do not intersect,
    // GetQuantity() returns 0, in which case the intersection type is
    // IT_EMPTY.  If the line and triangle intersect in a single point,
    // GetQuantity() returns 1, in which case the intersection type is
    // IT_POINT and GetPoint() returns the intersection point.  If the line
    // and triangle intersect in a segment, GetQuantity() returns 2, in which
    // case the intersection type is IT_SEGMENT and GetPoint() returns the
    // segment endpoints.
    int GetQuantity () const;
    const Vector2<Real>& GetPoint (int i) const;

private:
    using Intersector<Real,Vector2<Real> >::IT_EMPTY;
    using Intersector<Real,Vector2<Real> >::IT_POINT;
    using Intersector<Real,Vector2<Real> >::IT_SEGMENT;
    using Intersector<Real,Vector2<Real> >::m_iIntersectionType;

    // The objects to intersect.
    const Line2<Real>* m_pkLine;
    const Triangle2<Real>* m_pkTriangle;

    // Information about the intersection set.
    int m_iQuantity;
    Vector2<Real> m_akPoint[2];

// Internal use, shared by IntrRay2Triangle2 and IntrSegment2Triangle2.
public:
    // Determine how the triangle and line intersect (if at all).
    static void TriangleLineRelations (const Vector2<Real>& rkOrigin,
        const Vector2<Real>& rkDirection, const Triangle2<Real>& rkTriangle,
        Real afDist[3], int aiSign[3], int& riPositive, int& riNegative,
        int& riZero);

    // Compute the parameter interval for the segment of intersection when
    // the triangle transversely intersects the line.
    static void GetInterval (const Vector2<Real>& rkOrigin,
        const Vector2<Real>& rkDirection, const Triangle2<Real>& rkTriangle,
        const Real afDist[3], const int aiSign[3], Real afParam[2]);
};

typedef IntrLine2Triangle2<float> IntrLine2Triangle2f;
typedef IntrLine2Triangle2<double> IntrLine2Triangle2d;

}

#endif
