// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTRBOX2BOX2_H
#define WM4INTRBOX2BOX2_H

#include "Wm4FoundationLIB.h"
#include "Wm4Intersector.h"
#include "Wm4Box2.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntrBox2Box2
    : public Intersector<Real,Vector2<Real> >
{
public:
    IntrBox2Box2 (const Box2<Real>& rkBox0, const Box2<Real>& rkBox1);

    // object access
    const Box2<Real>& GetBox0 () const;
    const Box2<Real>& GetBox1 () const;

    // static test-intersection query
    virtual bool Test ();

private:
    // the objects to intersect
    const Box2<Real>* m_pkBox0;
    const Box2<Real>* m_pkBox1;
};

typedef IntrBox2Box2<float> IntrBox2Box2f;
typedef IntrBox2Box2<double> IntrBox2Box2d;

}

#endif
