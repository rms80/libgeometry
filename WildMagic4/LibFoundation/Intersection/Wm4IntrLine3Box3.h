// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTRLINE3BOX3_H
#define WM4INTRLINE3BOX3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Intersector.h"
#include "Wm4Line3.h"
#include "Wm4Box3.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntrLine3Box3
    : public Intersector<Real,Vector3<Real> >
{
public:
    IntrLine3Box3 (const Line3<Real>& rkLine, const Box3<Real>& rkBox);

    // object access
    const Line3<Real>& GetLine () const;
    const Box3<Real>& GetBox () const;

    // static intersection queries
    virtual bool Test ();
    virtual bool Find ();

    // the intersection set
    int GetQuantity () const;
    const Vector3<Real>& GetPoint (int i) const;

private:
    using Intersector<Real,Vector3<Real> >::IT_EMPTY;
    using Intersector<Real,Vector3<Real> >::IT_POINT;
    using Intersector<Real,Vector3<Real> >::IT_SEGMENT;
    using Intersector<Real,Vector3<Real> >::m_iIntersectionType;

    static bool Clip (Real fDenom, Real fNumer, Real& rfT0, Real& rfT1);

    // the objects to intersect
    const Line3<Real>* m_pkLine;
    const Box3<Real>* m_pkBox;

    // information about the intersection set
    int m_iQuantity;
    Vector3<Real> m_akPoint[2];

// internal use (shared by IntrRay3Box3 and IntrSegment3Box3)
public:
    static bool DoClipping (Real fT0, Real fT1, const Vector3<Real>& rkOrigin,
        const Vector3<Real>& rkDirection, const Box3<Real>& rkBox,
        bool bSolid, int& riQuantity, Vector3<Real> akPoint[2],
        int& riIntrType);
};

typedef IntrLine3Box3<float> IntrLine3Box3f;
typedef IntrLine3Box3<double> IntrLine3Box3d;

}

#endif
