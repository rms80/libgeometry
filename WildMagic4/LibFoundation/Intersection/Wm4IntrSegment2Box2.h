// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTRSEGMENT2BOX2_H
#define WM4INTRSEGMENT2BOX2_H

#include "Wm4FoundationLIB.h"
#include "Wm4Intersector.h"
#include "Wm4Segment2.h"
#include "Wm4Box2.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntrSegment2Box2
    : public Intersector<Real,Vector2<Real> >
{
public:
    IntrSegment2Box2 (const Segment2<Real>& rkSegment,
        const Box2<Real>& rkBox, bool bSolid);

    // object access
    const Segment2<Real>& GetSegment () const;
    const Box2<Real>& GetBox () const;

    // static intersection queries
    virtual bool Test ();
    virtual bool Find ();

    // the intersection set
    int GetQuantity () const;
    const Vector2<Real>& GetPoint (int i) const;

private:
    using Intersector<Real,Vector2<Real> >::m_iIntersectionType;

    // the objects to intersect
    const Segment2<Real>* m_pkSegment;
    const Box2<Real>* m_pkBox;
    bool m_bSolid;

    // information about the intersection set
    int m_iQuantity;
    Vector2<Real> m_akPoint[2];
};

typedef IntrSegment2Box2<float> IntrSegment2Box2f;
typedef IntrSegment2Box2<double> IntrSegment2Box2d;

}

#endif
