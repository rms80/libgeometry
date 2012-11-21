// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

//----------------------------------------------------------------------------
template <class Real>
AxisAlignedBox2<Real>::AxisAlignedBox2 ()
{
    // uninitialized
}
//----------------------------------------------------------------------------
template <class Real>
AxisAlignedBox2<Real>::AxisAlignedBox2 (Real fXMin, Real fXMax, Real fYMin,
    Real fYMax)
{
    Min[0] = fXMin;
    Max[0] = fXMax;
    Min[1] = fYMin;
    Max[1] = fYMax;
}
//----------------------------------------------------------------------------
template <class Real>
bool AxisAlignedBox2<Real>::HasXOverlap (const AxisAlignedBox2& rkBox) const
{
    return Max[0] >= rkBox.Min[0] && Min[0] <= rkBox.Max[0];
}
//----------------------------------------------------------------------------
template <class Real>
bool AxisAlignedBox2<Real>::HasYOverlap (const AxisAlignedBox2& rkBox) const
{
    return Max[1] >= rkBox.Min[1] && Min[1] <= rkBox.Max[1];
}
//----------------------------------------------------------------------------
template <class Real>
bool AxisAlignedBox2<Real>::TestIntersection (const AxisAlignedBox2& rkBox)
    const
{
    for (int i = 0; i < 2; i++)
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
bool AxisAlignedBox2<Real>::FindIntersection (const AxisAlignedBox2& rkBox,
    AxisAlignedBox2& rkIntr) const
{
    int i;
    for (i = 0; i < 2; i++)
    {
        if (Max[i] < rkBox.Min[i] || Min[i] > rkBox.Max[i])
        {
            return false;
        }
    }

    for (i = 0; i < 2; i++)
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
