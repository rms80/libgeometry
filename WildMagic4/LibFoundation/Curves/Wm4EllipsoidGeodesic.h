// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4ELLIPSOIDGEODESIC_H
#define WM4ELLIPSOIDGEODESIC_H

#include "Wm4FoundationLIB.h"
#include "Wm4RiemannianGeodesic.h"
#include "Wm4Vector3.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM EllipsoidGeodesic : public RiemannianGeodesic<Real>
{
public:
    // The ellipsoid is (x/a)^2 + (y/b)^2 + (z/c)^2 = 1, where fXExtent is
    // 'a', fYExtent is 'b', and fZExtent is 'c'.  The surface is represented
    // parametrically by angles u and v, say P(u,v) = (x(u,v),y(u,v),z(u,v)),
    //   P(u,v) =(a*cos(u)*sin(v), b*sin(u)*sin(v), c*cos(v))
    // with 0 <= u < 2*pi and 0 <= v <= pi.  The first-order derivatives are
    //   dP/du = (-a*sin(u)*sin(v), b*cos(u)*sin(v), 0)
    //   dP/dv = (a*cos(u)*cos(v), b*sin(u)*cos(v), -c*sin(v))
    // The metric tensor elements are
    //   g_{00} = Dot(dP/du,dP/du)
    //   g_{01} = Dot(dP/du,dP/dv)
    //   g_{10} = g_{01}
    //   g_{11} = Dot(dP/dv,dP/dv)

    EllipsoidGeodesic (Real fXExtent, Real fYExtent, Real fZExtent);
    virtual ~EllipsoidGeodesic ();

    Vector3<Real> ComputePosition (const GVector<Real>& rkPoint);
    virtual void ComputeMetric (const GVector<Real>& rkPoint);
    virtual void ComputeChristoffel1 (const GVector<Real>& rkPoint);

    // To compute the geodesic path connecting two parameter points (u0,v0)
    // and (u1,v1):
    //
    // float fA, fB, fC;  // the extents of the ellipsoid
    // EllipsoidGeodesic<float> kEG(fA,fB,fC);
    // GVectorf kParam0(2), kParam1(2);
    // kParam0[0] = u0;
    // kParam0[1] = v0;
    // kParam1[0] = u1;
    // kParam1[1] = v1;
    //
    // int iQuantity;
    // GVectorf* akPath;
    // kEG.ComputeGeodesic(kParam0,kParam1,iCurrQuantity,akPath);

private:
    using RiemannianGeodesic<Real>::m_kMetric;
    using RiemannianGeodesic<Real>::m_akChristoffel1;

    Real m_fXExtent, m_fYExtent, m_fZExtent;
};

typedef EllipsoidGeodesic<float> EllipsoidGeodesicf;
typedef EllipsoidGeodesic<double> EllipsoidGeodesicd;

}

#endif
