// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTRLINE2CIRCLE2_H
#define WM4INTRLINE2CIRCLE2_H

#include "Wm4FoundationLIB.h"
#include "Wm4Intersector.h"
#include "Wm4Line2.h"
#include "Wm4Circle2.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntrLine2Circle2
    : public Intersector<Real,Vector2<Real> >
{
public:
    IntrLine2Circle2 (const Line2<Real>& rkLine,
        const Circle2<Real>& rkCircle);

    // object access
    const Line2<Real>& GetLine () const;
    const Circle2<Real>& GetCircle () const;

    // static intersection query
    virtual bool Find ();

    // the intersection set
    int GetQuantity () const;
    const Vector2<Real>& GetPoint (int i) const;

private:
    using Intersector<Real,Vector2<Real> >::IT_EMPTY;
    using Intersector<Real,Vector2<Real> >::IT_POINT;
    using Intersector<Real,Vector2<Real> >::m_iIntersectionType;

    // the objects to intersect
    const Line2<Real>* m_pkLine;
    const Circle2<Real>* m_pkCircle;

    // information about the intersection set
    int m_iQuantity;
    Vector2<Real> m_akPoint[2];

// internal use (shared by IntrRay2Circle2, IntrSegment2Circle2,
//   IntrLine2Arc2, IntrRay2Arc2, IntrSegment2Arc2)
public:
    static bool Find (const Vector2<Real>& rkOrigin,
        const Vector2<Real>& rkDirection, const Vector2<Real>& rkCenter,
        Real fRadius, int& riRootCount, Real afT[2]);
};

typedef IntrLine2Circle2<float> IntrLine2Circle2f;
typedef IntrLine2Circle2<double> IntrLine2Circle2d;

}

#endif
