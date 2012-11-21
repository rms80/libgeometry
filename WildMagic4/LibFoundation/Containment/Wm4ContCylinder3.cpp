// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4ContCylinder3.h"
#include "Wm4ApprLineFit3.h"
#include "Wm4DistVector3Line3.h"

namespace Wm4
{
//----------------------------------------------------------------------------
template <class Real>
Cylinder3<Real> ContCylinder (int iQuantity, const Vector3<Real>* akPoint)
{
    Cylinder3<Real> kCylinder;

    Line3<Real> kLine = OrthogonalLineFit3(iQuantity,akPoint);

    Real fMaxRadiusSqr = (Real)0.0;
    int i;
    for (i = 0; i < iQuantity; i++)
    {
        Real fRadiusSqr = DistVector3Line3<Real>(akPoint[i],
            kLine).GetSquared();
        if (fRadiusSqr > fMaxRadiusSqr)
        {
            fMaxRadiusSqr = fRadiusSqr;
        }
    }

    Vector3<Real> kDiff = akPoint[0] - kLine.Origin;
    Real fWMin = kLine.Direction.Dot(kDiff), fWMax = fWMin;
    for (i = 1; i < iQuantity; i++)
    {
        kDiff = akPoint[i] - kLine.Origin;
        Real fW = kLine.Direction.Dot(kDiff);
        if (fW < fWMin)
        {
            fWMin = fW;
        }
        else if (fW > fWMax)
        {
            fWMax = fW;
        }
    }

    kCylinder.Segment.Origin = kLine.Origin +
        (((Real)0.5)*(fWMax+fWMin))*kLine.Direction;
    kCylinder.Segment.Direction = kLine.Direction;
    kCylinder.Radius = Math<Real>::Sqrt(fMaxRadiusSqr);
    kCylinder.Height = fWMax - fWMin;

    return kCylinder;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
template WM4_FOUNDATION_ITEM
Cylinder3<float> ContCylinder<float> (int, const Vector3<float>*);

template WM4_FOUNDATION_ITEM
Cylinder3<double> ContCylinder<double> (int, const Vector3<double>*);
//----------------------------------------------------------------------------
}
