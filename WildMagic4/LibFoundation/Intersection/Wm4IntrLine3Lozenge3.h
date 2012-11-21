// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTRLINE3LOZENGE3_H
#define WM4INTRLINE3LOZENGE3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Intersector.h"
#include "Wm4Line3.h"
#include "Wm4Lozenge3.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntrLine3Lozenge3
    : public Intersector<Real,Vector3<Real> >
{
public:
    IntrLine3Lozenge3 (const Line3<Real>& rkLine,
        const Lozenge3<Real>& rkLozenge);

    // object access
    const Line3<Real>& GetLine () const;
    const Lozenge3<Real>& GetLozenge () const;

    // static intersection query
    virtual bool Test ();

private:
    // the objects to intersect
    const Line3<Real>* m_pkLine;
    const Lozenge3<Real>* m_pkLozenge;
};

typedef IntrLine3Lozenge3<float> IntrLine3Lozenge3f;
typedef IntrLine3Lozenge3<double> IntrLine3Lozenge3d;

}

#endif
