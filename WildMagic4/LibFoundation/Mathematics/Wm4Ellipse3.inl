// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

//----------------------------------------------------------------------------
template <class Real>
Ellipse3<Real>::Ellipse3 ()
{
    // uninitialized
}
//----------------------------------------------------------------------------
template <class Real>
Ellipse3<Real>::Ellipse3 (const Vector3<Real>& rkCenter,
    const Vector3<Real>& rkNormal, const Vector3<Real>& rkMajor,
    const Vector3<Real>& rkMinor, Real fMajorLength, Real fMinorLength)
    :
    Center(rkCenter),
    Normal(rkNormal),
    Major(rkMajor),
    Minor(rkMinor)
{
    MajorLength = fMajorLength;
    MinorLength = fMinorLength;
}
//----------------------------------------------------------------------------
