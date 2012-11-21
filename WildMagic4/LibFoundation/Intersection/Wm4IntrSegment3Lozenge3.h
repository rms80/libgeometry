// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTRSEGMENT3LOZENGE3_H
#define WM4INTRSEGMENT3LOZENGE3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Intersector.h"
#include "Wm4Segment3.h"
#include "Wm4Lozenge3.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntrSegment3Lozenge3
    : public Intersector<Real,Vector3<Real> >
{
public:
    IntrSegment3Lozenge3 (const Segment3<Real>& rkSegment,
        const Lozenge3<Real>& rkLozenge);

    // object access
    const Segment3<Real>& GetSegment () const;
    const Lozenge3<Real>& GetLozenge () const;

    // static intersection query
    virtual bool Test ();

private:
    // the objects to intersect
    const Segment3<Real>* m_pkSegment;
    const Lozenge3<Real>* m_pkLozenge;
};

typedef IntrSegment3Lozenge3<float> IntrSegment3Lozenge3f;
typedef IntrSegment3Lozenge3<double> IntrSegment3Lozenge3d;

}

#endif
