// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

//----------------------------------------------------------------------------
template <int ISIZE>
RVector2<ISIZE>::RVector2 ()
{
    // the vector is uninitialized
}
//----------------------------------------------------------------------------
template <int ISIZE>
RVector2<ISIZE>::RVector2 (const RVector2& rkV)
{
    m_akTuple[0] = rkV.m_akTuple[0];
    m_akTuple[1] = rkV.m_akTuple[1];
}
//----------------------------------------------------------------------------
#ifndef WM4_USING_VC70
template <int ISIZE>
RVector2<ISIZE>::RVector2 (const TRVector<2,ISIZE>& rkV)
{
    m_akTuple[0] = rkV[0];
    m_akTuple[1] = rkV[1];
}
#endif
//----------------------------------------------------------------------------
template <int ISIZE>
RVector2<ISIZE>::RVector2 (const TRational<ISIZE>& rkX,
    const TRational<ISIZE>& rkY)
{
    m_akTuple[0] = rkX;
    m_akTuple[1] = rkY;
}
//----------------------------------------------------------------------------
template <int ISIZE>
RVector2<ISIZE>& RVector2<ISIZE>::operator= (const RVector2& rkV)
{
    m_akTuple[0] = rkV.m_akTuple[0];
    m_akTuple[1] = rkV.m_akTuple[1];
    return *this;
}
//----------------------------------------------------------------------------
#ifndef WM4_USING_VC70
template <int ISIZE>
RVector2<ISIZE>& RVector2<ISIZE>::operator= (const TRVector<2,ISIZE>& rkV)
{
    m_akTuple[0] = rkV[0];
    m_akTuple[1] = rkV[1];
    return *this;
}
#endif
//----------------------------------------------------------------------------
template <int ISIZE>
TRational<ISIZE> RVector2<ISIZE>::X () const
{
    return m_akTuple[0];
}
//----------------------------------------------------------------------------
template <int ISIZE>
TRational<ISIZE>& RVector2<ISIZE>::X ()
{
    return m_akTuple[0];
}
//----------------------------------------------------------------------------
template <int ISIZE>
TRational<ISIZE> RVector2<ISIZE>::Y () const
{
    return m_akTuple[1];
}
//----------------------------------------------------------------------------
template <int ISIZE>
TRational<ISIZE>& RVector2<ISIZE>::Y ()
{
    return m_akTuple[1];
}
//----------------------------------------------------------------------------
template <int ISIZE>
TRational<ISIZE> RVector2<ISIZE>::Dot (const RVector2& rkV) const
{
    return m_akTuple[0]*rkV.m_akTuple[0] + m_akTuple[1]*rkV.m_akTuple[1];
}
//----------------------------------------------------------------------------
template <int ISIZE>
RVector2<ISIZE> RVector2<ISIZE>::Perp () const
{
    return RVector2<ISIZE>(m_akTuple[1],-m_akTuple[0]);
}
//----------------------------------------------------------------------------
template <int ISIZE>
TRational<ISIZE> RVector2<ISIZE>::DotPerp (const RVector2& rkV) const
{
    return m_akTuple[0]*rkV.m_akTuple[1] - m_akTuple[1]*rkV.m_akTuple[0];
}
//----------------------------------------------------------------------------
