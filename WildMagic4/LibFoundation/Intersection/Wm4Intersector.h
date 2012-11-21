// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTERSECTOR_H
#define WM4INTERSECTOR_H

#include "Wm4FoundationLIB.h"
#include "Wm4Vector2.h"
#include "Wm4Vector3.h"

namespace Wm4
{

template <class Real, class TVector>
class WM4_FOUNDATION_ITEM Intersector
{
public:
    // abstract base class
    virtual ~Intersector ();

    // Static intersection queries.  The default implementations return
    // 'false'.  The Find query produces a set of intersection.  The derived
    // class is responsible for providing access to that set, since the nature
    // of the set is dependent on the object types.
    virtual bool Test ();
    virtual bool Find ();

    // Dynamic intersection queries.  The default implementations return
    // 'false'.  The Find query produces a set of first contact.  The derived
    // class is responsible for providing access to that set, since the nature
    // of the set is dependent on the object types.
    virtual bool Test (Real fTMax, const TVector& rkVelocity0,
        const TVector& rkVelocity1);
    virtual bool Find (Real fTMax, const TVector& rkVelocity0,
        const TVector& rkVelocity1);

    // The time at which two objects are in first contact for the dynamic
    // intersection queries.
    Real GetContactTime () const;

    // information about the intersection set
    enum
    {
        IT_EMPTY,
        IT_POINT,
        IT_SEGMENT,
        IT_RAY,
        IT_LINE,
        IT_POLYGON,
        IT_PLANE,
        IT_POLYHEDRON,
        IT_OTHER
    };
    int GetIntersectionType () const;

protected:
    Intersector ();

    Real m_fContactTime;
    int m_iIntersectionType;
};

typedef Intersector<float, Vector2<float> > Intersector2f;
typedef Intersector<float, Vector3<float> > Intersector3f;
typedef Intersector<double, Vector2<double> > Intersector2d;
typedef Intersector<double, Vector3<double> > Intersector3d;

}

#endif
