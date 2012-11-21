// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4IMPLICITSURFACE_H
#define WM4IMPLICITSURFACE_H

#include "Wm4FoundationLIB.h"
#include "Wm4Matrix3.h"
#include "Wm4Surface.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM ImplicitSurface : public Surface<Real>
{
public:
    // Surface is defined by F(x,y,z) = 0.  In all member functions it is
    // the application's responsibility to ensure that (x,y,z) is a solution
    // to F = 0.

    // abstract base class
    virtual ~ImplicitSurface ();

    // the function
    virtual Real F (const Vector3<Real>& rkP) const = 0;

    // first-order partial derivatives
    virtual Real FX (const Vector3<Real>& rkP) const = 0;
    virtual Real FY (const Vector3<Real>& rkP) const = 0;
    virtual Real FZ (const Vector3<Real>& rkP) const = 0;

    // second-order partial derivatives
    virtual Real FXX (const Vector3<Real>& rkP) const = 0;
    virtual Real FXY (const Vector3<Real>& rkP) const = 0;
    virtual Real FXZ (const Vector3<Real>& rkP) const = 0;
    virtual Real FYY (const Vector3<Real>& rkP) const = 0;
    virtual Real FYZ (const Vector3<Real>& rkP) const = 0;
    virtual Real FZZ (const Vector3<Real>& rkP) const = 0;

    // verify point is on surface (within the tolerance specified by epsilon)
    bool IsOnSurface (const Vector3<Real>& rkP, Real fEpsilon) const;

    // first-order derivatives
    Vector3<Real> GetGradient (const Vector3<Real>& rkP) const;

    // second-order derivatives
    Matrix3<Real> GetHessian (const Vector3<Real>& rkP) const;

    // Compute a coordinate frame.  The set {T0,T1,N} is a right-handed
    // orthonormal set.
    void GetFrame (const Vector3<Real>& rkP, Vector3<Real>& rkTangent0,
        Vector3<Real>& rkTangent1, Vector3<Real>& rkNormal) const;

    // Differential geometric quantities.  The returned scalars are the
    // principal curvatures and the returned vectors are the corresponding
    // principal directions.
    bool ComputePrincipalCurvatureInfo (const Vector3<Real>& rkP,
        Real& rfCurv0, Real& rfCurv1, Vector3<Real>& rkDir0,
        Vector3<Real>& rkDir1);

protected:
    ImplicitSurface ();
};

typedef ImplicitSurface<float> ImplicitSurfacef;
typedef ImplicitSurface<double> ImplicitSurfaced;

}

#endif
