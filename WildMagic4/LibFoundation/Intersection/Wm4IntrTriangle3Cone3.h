// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTRTRIANGLE3CONE3_H
#define WM4INTRTRIANGLE3CONE3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Intersector.h"
#include "Wm4Triangle3.h"
#include "Wm4Cone3.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntrTriangle3Cone3
    : public Intersector<Real,Vector3<Real> >
{
public:
    IntrTriangle3Cone3 (const Triangle3<Real>& rkTriangle,
        const Cone3<Real>& rkCone);

    // object access
    const Triangle3<Real>& GetTriangle () const;
    const Cone3<Real>& GetCone () const;

    // static query
    virtual bool Test ();

private:
    // the objects to intersect
    const Triangle3<Real>* m_pkTriangle;
    const Cone3<Real>* m_pkCone;
};

typedef IntrTriangle3Cone3<float> IntrTriangle3Cone3f;
typedef IntrTriangle3Cone3<double> IntrTriangle3Cone3d;

}

#endif
