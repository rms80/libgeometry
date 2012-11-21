// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTRRAY3LOZENGE3_H
#define WM4INTRRAY3LOZENGE3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Intersector.h"
#include "Wm4Ray3.h"
#include "Wm4Lozenge3.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntrRay3Lozenge3
    : public Intersector<Real,Vector3<Real> >
{
public:
    IntrRay3Lozenge3 (const Ray3<Real>& rkRay,
        const Lozenge3<Real>& rkLozenge);

    // object access
    const Ray3<Real>& GetRay () const;
    const Lozenge3<Real>& GetLozenge () const;

    // static intersection query
    virtual bool Test ();

private:
    // the objects to intersect
    const Ray3<Real>* m_pkRay;
    const Lozenge3<Real>* m_pkLozenge;
};

typedef IntrRay3Lozenge3<float> IntrRay3Lozenge3f;
typedef IntrRay3Lozenge3<double> IntrRay3Lozenge3d;

}

#endif
