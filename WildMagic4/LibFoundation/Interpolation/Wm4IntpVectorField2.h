// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4INTPVECTORFIELD2_H
#define WM4INTPVECTORFIELD2_H

// Given points (x0[i],y0[i]) which are mapped to (x1[i],y1[i]) for
// 0 <= i < N, interpolate positions (xIn,yIn) to (xOut,yOut).

#include "Wm4FoundationLIB.h"
#include "Wm4IntpQdrNonuniform2.h"

namespace Wm4
{

template <class Real>
class WM4_FOUNDATION_ITEM IntpVectorField2
{
public:
    // Construction and destruction.  If you want IntpVectorField2 to delete
    // the input arrays during destruction, set bOwner to 'true'.  Otherwise,
    // you own the arrays and must delete them yourself.
    //
    // The computation type is for the Delaunay triangulation and should be
    // one of Query::{QT_INT64,QT_INTEGER,QT_RATIONAL,QT_REAL}.
    IntpVectorField2 (int iQuantity, Vector2<Real>* akDomain,
        Vector2<Real>* akRange, bool bOwner, Query::Type eQueryType);

    ~IntpVectorField2 ();

    // Return 'true' if and only if (xIn,yIn) is in the convex hull of the
    // input points.  In this case, (xOut,yOut) is valid.  If the return
    // value is 'false', (xOut,yOut) is invalid and should not be used.
    bool Evaluate (const Vector2<Real>& rkInput, Vector2<Real>& rkOutput); 

protected:
    Delaunay2<Real>* m_pkDel;
    IntpQdrNonuniform2<Real>* m_pkXInterp;
    IntpQdrNonuniform2<Real>* m_pkYInterp;
};

typedef IntpVectorField2<float> IntpVectorField2f;
typedef IntpVectorField2<double> IntpVectorField2d;

}

#endif
