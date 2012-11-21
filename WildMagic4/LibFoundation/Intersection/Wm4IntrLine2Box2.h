// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTRLINE2BOX2_H
#define WM4INTRLINE2BOX2_H

#include "Wm4FoundationLIB.h"
#include "Wm4Intersector.h"
#include "Wm4Line2.h"
#include "Wm4Box2.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntrLine2Box2
    : public Intersector<Real,Vector2<Real> >
{
public:
    IntrLine2Box2 (const Line2<Real>& rkLine, const Box2<Real>& rkBox);

    // object access
    const Line2<Real>& GetLine () const;
    const Box2<Real>& GetBox () const;

    // static intersection queries
    virtual bool Test ();
    virtual bool Find ();

    // the intersection set
    int GetQuantity () const;
    const Vector2<Real>& GetPoint (int i) const;

private:
    using Intersector<Real,Vector2<Real> >::IT_EMPTY;
    using Intersector<Real,Vector2<Real> >::IT_POINT;
    using Intersector<Real,Vector2<Real> >::IT_SEGMENT;
    using Intersector<Real,Vector2<Real> >::m_iIntersectionType;

    static bool Clip (Real fDenom, Real fNumer, Real& rfT0, Real& rfT1);

    // the objects to intersect
    const Line2<Real>* m_pkLine;
    const Box2<Real>* m_pkBox;

    // information about the intersection set
    int m_iQuantity;
    Vector2<Real> m_akPoint[2];

// internal use (shared by IntrRay2Box2 and IntrSegment2Box2)
public:
    static bool DoClipping (Real fT0, Real fT1, const Vector2<Real>& rkOrigin,
        const Vector2<Real>& rkDirection, const Box2<Real>& rkBox,
        bool bSolid, int& riQuantity, Vector2<Real> akPoint[2],
        int& riIntrType);
};

typedef IntrLine2Box2<float> IntrLine2Box2f;
typedef IntrLine2Box2<double> IntrLine2Box2d;

}

#endif
