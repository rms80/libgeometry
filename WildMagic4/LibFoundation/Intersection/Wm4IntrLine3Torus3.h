// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTRLINE3TORUS3_H
#define WM4INTRLINE3TORUS3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Intersector.h"
#include "Wm4Line3.h"
#include "Wm4Torus3.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntrLine3Torus3
    : public Intersector<Real,Vector3<Real> >
{
public:
    IntrLine3Torus3 (const Line3<Real>& rkLine, const Torus3<Real>& rkTorus);

    // object access
    const Line3<Real>& GetLine () const;
    const Torus3<Real>& GetTorus () const;

    // static intersection query
    virtual bool Find ();

    // the intersection set (quantity is at most 4)
    int GetQuantity () const;
    const Vector3<Real>& GetPoint (int i) const;

private:
    using Intersector<Real,Vector3<Real> >::IT_EMPTY;
    using Intersector<Real,Vector3<Real> >::IT_POINT;
    using Intersector<Real,Vector3<Real> >::m_iIntersectionType;

    // the objects to intersect
    const Line3<Real>* m_pkLine;
    const Torus3<Real>* m_pkTorus;

    // information about the intersection set
    int m_iQuantity;
    Vector3<Real> m_akPoint[4];
};

typedef IntrLine3Torus3<float> IntrLine3Torus3f;
typedef IntrLine3Torus3<double> IntrLine3Torus3d;

}

#endif
