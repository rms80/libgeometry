// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

//----------------------------------------------------------------------------
inline IVector2::IVector2 ()
{
    // the vector is uninitialized
}
//----------------------------------------------------------------------------
inline IVector2::IVector2 (const IVector2& rkV)
{
    m_aiTuple[0] = rkV.m_aiTuple[0];
    m_aiTuple[1] = rkV.m_aiTuple[1];
}
//----------------------------------------------------------------------------
inline IVector2::IVector2 (const TIVector<2>& rkV)
{
    m_aiTuple[0] = rkV[0];
    m_aiTuple[1] = rkV[1];
}
//----------------------------------------------------------------------------
inline IVector2::IVector2 (const Integer64& riX, const Integer64& riY)
{
    m_aiTuple[0] = riX;
    m_aiTuple[1] = riY;
}
//----------------------------------------------------------------------------
inline IVector2& IVector2::operator= (const IVector2& rkV)
{
    m_aiTuple[0] = rkV.m_aiTuple[0];
    m_aiTuple[1] = rkV.m_aiTuple[1];
    return *this;
}
//----------------------------------------------------------------------------
inline IVector2& IVector2::operator= (const TIVector<2>& rkV)
{
    m_aiTuple[0] = rkV[0];
    m_aiTuple[1] = rkV[1];
    return *this;
}
//----------------------------------------------------------------------------
inline Integer64 IVector2::X () const
{
    return m_aiTuple[0];
}
//----------------------------------------------------------------------------
inline Integer64& IVector2::X ()
{
    return m_aiTuple[0];
}
//----------------------------------------------------------------------------
inline Integer64 IVector2::Y () const
{
    return m_aiTuple[1];
}
//----------------------------------------------------------------------------
inline Integer64& IVector2::Y ()
{
    return m_aiTuple[1];
}
//----------------------------------------------------------------------------
inline Integer64 IVector2::Dot (const IVector2& rkV) const
{
    return m_aiTuple[0]*rkV.m_aiTuple[0] + m_aiTuple[1]*rkV.m_aiTuple[1];
}
//----------------------------------------------------------------------------
inline IVector2 IVector2::Perp () const
{
    return IVector2(m_aiTuple[1],-m_aiTuple[0]);
}
//----------------------------------------------------------------------------
inline Integer64 IVector2::DotPerp (const IVector2& rkV) const
{
    return m_aiTuple[0]*rkV.m_aiTuple[1] - m_aiTuple[1]*rkV.m_aiTuple[0];
}
//----------------------------------------------------------------------------
