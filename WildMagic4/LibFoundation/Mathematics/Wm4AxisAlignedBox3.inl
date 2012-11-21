// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

//----------------------------------------------------------------------------
template <class Real>
AxisAlignedBox3<Real>::AxisAlignedBox3 ()
{
    // uninitialized
}
//----------------------------------------------------------------------------
template <class Real>
AxisAlignedBox3<Real>::AxisAlignedBox3 (Real fXMin, Real fXMax, Real fYMin,
    Real fYMax, Real fZMin, Real fZMax)
{
    Min[0] = fXMin;
    Max[0] = fXMax;
    Min[1] = fYMin;
    Max[1] = fYMax;
    Min[2] = fZMin;
    Max[2] = fZMax;
}
//----------------------------------------------------------------------------
template <class Real>
bool AxisAlignedBox3<Real>::HasXOverlap (const AxisAlignedBox3& rkBox) const
{
    return Max[0] >= rkBox.Min[0] && Min[0] <= rkBox.Max[0];
}
//----------------------------------------------------------------------------
template <class Real>
bool AxisAlignedBox3<Real>::HasYOverlap (const AxisAlignedBox3& rkBox) const
{
    return Max[1] >= rkBox.Min[1] && Min[1] <= rkBox.Max[1];
}
//----------------------------------------------------------------------------
template <class Real>
bool AxisAlignedBox3<Real>::HasZOverlap (const AxisAlignedBox3& rkBox) const
{
    return Max[2] >= rkBox.Min[2] && Min[2] <= rkBox.Max[2];
}
//----------------------------------------------------------------------------
template <class Real>
bool AxisAlignedBox3<Real>::TestIntersection (const AxisAlignedBox3& rkBox)
    const
{
    for (int i = 0; i < 3; i++)
    {
        if (Max[i] < rkBox.Min[i] || Min[i] > rkBox.Max[i])
        {
            return false;
        }
    }
    return true;
}
//----------------------------------------------------------------------------
template <class Real>
bool AxisAlignedBox3<Real>::FindIntersection (const AxisAlignedBox3& rkBox,
    AxisAlignedBox3& rkIntr) const
{
    int i;
    for (i = 0; i < 3; i++)
    {
        if (Max[i] < rkBox.Min[i] || Min[i] > rkBox.Max[i])
        {
            return false;
        }
    }

    for (i = 0; i < 3; i++)
    {
        if (Max[i] <= rkBox.Max[i])
        {
            rkIntr.Max[i] = Max[i];
        }
        else
        {
            rkIntr.Max[i] = rkBox.Max[i];
        }

        if (Min[i] <= rkBox.Min[i])
        {
            rkIntr.Min[i] = rkBox.Min[i];
        }
        else
        {
            rkIntr.Min[i] = Min[i];
        }
    }
    return true;
}
//----------------------------------------------------------------------------
