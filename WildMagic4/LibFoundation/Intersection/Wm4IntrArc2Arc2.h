// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTRARC2ARC2_H
#define WM4INTRARC2ARC2_H

#include "Wm4FoundationLIB.h"
#include "Wm4Intersector.h"
#include "Wm4Arc2.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntrArc2Arc2
    : public Intersector<Real,Vector2<Real> >
{
public:
    IntrArc2Arc2 (const Arc2<Real>& rkArc0, const Arc2<Real>& rkArc1);

    // object access
    const Arc2<Real>& GetArc0 () const;
    const Arc2<Real>& GetArc1 () const;

    // static find-intersection query
    virtual bool Find ();

    // Intersection set for static find-intersection query.  The quantity Q is
    // 0, 1, or 2.  When Q > 0, the interpretation depends on the intersection
    // type.
    //   IT_POINT:  Q distinct points of intersection
    //   IT_OTHER:  The arcs overlap by more than a point.  The arc of
    //              intersection is accessed by GetIntersectionArc().  Q is
    //              invalid.
    int GetQuantity () const;
    const Vector2<Real>& GetPoint (int i) const;
    const Arc2<Real>& GetIntersectionArc () const;

private:
    using Intersector<Real,Vector2<Real> >::IT_EMPTY;
    using Intersector<Real,Vector2<Real> >::IT_POINT;
    using Intersector<Real,Vector2<Real> >::IT_OTHER;
    using Intersector<Real,Vector2<Real> >::m_iIntersectionType;

    // the objects to intersect
    const Arc2<Real>* m_pkArc0;
    const Arc2<Real>* m_pkArc1;

    // set of intersection
    int m_iQuantity;
    Vector2<Real> m_akPoint[2];
    Arc2<Real> m_kIntersectionArc;
};

typedef IntrArc2Arc2<float> IntrArc2Arc2f;
typedef IntrArc2Arc2<double> IntrArc2Arc2d;

}

#endif
