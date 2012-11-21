// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTRTETRAHEDRON3TETRAHEDRON3_H
#define WM4INTRTETRAHEDRON3TETRAHEDRON3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Intersector.h"
#include "Wm4Tetrahedron3.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntrTetrahedron3Tetrahedron3
    : public Intersector<Real,Vector3<Real> >
{
public:
    IntrTetrahedron3Tetrahedron3 (const Tetrahedron3<Real>& rkTetrahedron0,
        const Tetrahedron3<Real>& rkTetrahedron1);

    // object access
    const Tetrahedron3<Real>& GetTetrahedron0 () const;
    const Tetrahedron3<Real>& GetTetrahedron1 () const;

    // static query
    virtual bool Find ();

    // information about the intersection set
    const std::vector<Tetrahedron3<Real> >& GetIntersection () const;

private:
    static void SplitAndDecompose (Tetrahedron3<Real> kTetra,
        const Plane3<Real>& rkPlane,
        std::vector<Tetrahedron3<Real> >& rkInside);

    // the objects to intersect
    const Tetrahedron3<Real>* m_pkTetrahedron0;
    const Tetrahedron3<Real>* m_pkTetrahedron1;

    std::vector<Tetrahedron3<Real> > m_kIntersection;
};

typedef IntrTetrahedron3Tetrahedron3<float> IntrTetrahedron3Tetrahedron3f;
typedef IntrTetrahedron3Tetrahedron3<double> IntrTetrahedron3Tetrahedron3d;

}

#endif
