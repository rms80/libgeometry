// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTERSECTOR1_H
#define WM4INTERSECTOR1_H

#include "Wm4FoundationLIB.h"
#include "Wm4Math.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM Intersector1
{
public:
    // A class for intersection of intervals [u0,u1] and [v0,v1].  The end
    // points must be ordered:  u0 <= u1 and v0 <= v1.  Values of MAX_REAL
    // and -MAX_REAL are allowed, and degenerate intervals are allowed:
    // u0 = u1 or v0 = v1.
    Intersector1 (Real fU0, Real fU1, Real fV0, Real fV1);
    Intersector1 (Real afU[2], Real afV[2]);
    virtual ~Intersector1 ();

    // object access
    Real GetU (int i) const;
    Real GetV (int i) const;

    // static intersection queries
    virtual bool Test ();
    virtual bool Find ();

    // Dynamic intersection queries.  The Find query produces a set of first
    // contact.
    virtual bool Test (Real fTMax, Real fSpeedU, Real fSpeedV);
    virtual bool Find (Real fTMax, Real fSpeedU, Real fSpeedV);

    // The time at which two intervals are in first/last contact for the
    // dynamic intersection queries.
    Real GetFirstTime () const;
    Real GetLastTime () const;

    // information about the intersection set
    int GetQuantity () const;
    Real GetOverlap (int i) const;

protected:
    // the intervals to intersect
    Real m_afU[2], m_afV[2];

    // information about the intersection set
    Real m_fFirstTime, m_fLastTime;
    int m_iQuantity;
    Real m_afOverlap[2];
};

typedef Intersector1<float> Intersector1f;
typedef Intersector1<double> Intersector1d;

}

#endif
