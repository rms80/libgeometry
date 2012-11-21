// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#ifndef WM4ELLIPSE3_H
#define WM4ELLIPSE3_H

#include "Wm4FoundationLIB.h"
#include "Wm4Vector3.h"

namespace Wm4
{

template <class Real>
class Ellipse3
{
public:
    // Plane containing ellipse is Dot(N,X-C) = 0 where X is any point in the
    // plane.  Vectors U, V, and N form an orthonormal right-handed set
    // (matrix [U V N] is orthonormal and has determinant 1).  Ellipse within
    // the plane is parameterized by X = C + a*cos(t)*U + b*sin(t)*V where
    // t is an angle in [0,2*pi) and where a >= b > 0.  The symbols in this
    // discussion are related to the class members as follows.  The member
    // 'Center' is C, 'Normal' is N, 'Major' is U, 'Minor' is V, 'MajorLength'
    // is a, and 'MinorLength' is b.

    // construction
    Ellipse3 ();  // uninitialized
    Ellipse3 (const Vector3<Real>& rkCenter, const Vector3<Real>& rkNormal,
        const Vector3<Real>& rkMajor, const Vector3<Real>& rkMinor, 
        Real fMajorLength, Real fMinorLength);

    Vector3<Real> Center, Normal, Major, Minor;
    Real MajorLength, MinorLength;
};

#include "Wm4Ellipse3.inl"

typedef Ellipse3<float> Ellipse3f;
typedef Ellipse3<double> Ellipse3d;

}

#endif
