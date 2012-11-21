// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4BSPLINESURFACEFIT_H
#define WM4BSPLINESURFACEFIT_H

#include "Wm4FoundationLIB.h"
#include "Wm4BSplineFitBasis.h"
#include "Wm4BandedMatrix.h"
#include "Wm4Vector3.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM BSplineSurfaceFit
{
public:
    // Construction and destruction.  The preconditions for calling the
    // constructor are
    //   1 <= iDegree0 && iDegree0 < iControlQuantity0 <= iSampleQuantity0
    //   1 <= iDegree1 && iDegree1 < iControlQuantity1 <= iSampleQuantity1
    BSplineSurfaceFit (int iDegree0, int iControlQuantity0,
        int iSampleQuantity0, int iDegree1, int iControlQuantity1,
        int iSampleQuantity1, Vector3<Real>** aakSamplePoint);
    ~BSplineSurfaceFit ();

    // Access to input sample information.
    int GetSampleQuantity (int i) const;
    Vector3<Real>** GetSamplePoints () const;

    // Access to output control point and surface information.
    int GetDegree (int i) const;
    int GetControlQuantity (int i) const;
    Vector3<Real>** GetControlPoints () const;
    const BSplineFitBasis<Real>& GetBasis (int i) const;

    // Evaluation of the B-spline surface.  It is defined for 0 <= u <= 1
    // and 0 <= v <= 1.  If a parameter value is outside [0,1], it is clamped
    // to [0,1].
    Vector3<Real> GetPosition (Real fU, Real fV) const;

private:
    // Input sample information.
    int m_aiSampleQuantity[2];
    Vector3<Real>** m_aakSamplePoint;

    // The fitted B-spline surface, open and with uniform knots.
    int m_aiDegree[2];
    int m_aiControlQuantity[2];
    Vector3<Real>** m_aakControlPoint;
    BSplineFitBasis<Real>* m_apkBasis[2];
};

typedef BSplineSurfaceFit<float> BSplineSurfaceFitf;
typedef BSplineSurfaceFit<double> BSplineSurfaceFitd;

}

#endif
