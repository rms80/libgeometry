// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

//----------------------------------------------------------------------------
inline IVector3::IVector3 ()
{
    // the vector is uninitialized
}
//----------------------------------------------------------------------------
inline IVector3::IVector3 (const IVector3& rkV)
{
    m_aiTuple[0] = rkV.m_aiTuple[0];
    m_aiTuple[1] = rkV.m_aiTuple[1];
    m_aiTuple[2] = rkV.m_aiTuple[2];
}
//----------------------------------------------------------------------------
inline IVector3::IVector3 (const TIVector<3>& rkV)
{
    m_aiTuple[0] = rkV[0];
    m_aiTuple[1] = rkV[1];
    m_aiTuple[2] = rkV[2];
}
//----------------------------------------------------------------------------
inline IVector3::IVector3 (const Integer64& riX, const Integer64& riY,
    const Integer64& riZ)
{
    m_aiTuple[0] = riX;
    m_aiTuple[1] = riY;
    m_aiTuple[2] = riZ;
}
//----------------------------------------------------------------------------
inline IVector3& IVector3::operator= (const IVector3& rkV)
{
    m_aiTuple[0] = rkV.m_aiTuple[0];
    m_aiTuple[1] = rkV.m_aiTuple[1];
    m_aiTuple[2] = rkV.m_aiTuple[2];
    return *this;
}
//----------------------------------------------------------------------------
inline IVector3& IVector3::operator= (const TIVector<3>& rkV)
{
    m_aiTuple[0] = rkV[0];
    m_aiTuple[1] = rkV[1];
    m_aiTuple[2] = rkV[2];
    return *this;
}
//----------------------------------------------------------------------------
inline Integer64 IVector3::X () const
{
    return m_aiTuple[0];
}
//----------------------------------------------------------------------------
inline Integer64& IVector3::X ()
{
    return m_aiTuple[0];
}
//----------------------------------------------------------------------------
inline Integer64 IVector3::Y () const
{
    return m_aiTuple[1];
}
//----------------------------------------------------------------------------
inline Integer64& IVector3::Y ()
{
    return m_aiTuple[1];
}
//----------------------------------------------------------------------------
inline Integer64 IVector3::Z () const
{
    return m_aiTuple[2];
}
//----------------------------------------------------------------------------
inline Integer64& IVector3::Z ()
{
    return m_aiTuple[2];
}
//----------------------------------------------------------------------------
inline Integer64 IVector3::Dot (const IVector3& rkV) const
{
    return m_aiTuple[0]*rkV.m_aiTuple[0] + m_aiTuple[1]*rkV.m_aiTuple[1] +
        m_aiTuple[2]*rkV.m_aiTuple[2];
}
//----------------------------------------------------------------------------
inline IVector3 IVector3::Cross (const IVector3& rkV) const
{
    return IVector3(
        m_aiTuple[1]*rkV.m_aiTuple[2] - m_aiTuple[2]*rkV.m_aiTuple[1],
        m_aiTuple[2]*rkV.m_aiTuple[0] - m_aiTuple[0]*rkV.m_aiTuple[2],
        m_aiTuple[0]*rkV.m_aiTuple[1] - m_aiTuple[1]*rkV.m_aiTuple[0]);
}
//----------------------------------------------------------------------------
inline Integer64 IVector3::TripleScalar (const IVector3& rkU,
    const IVector3& rkV) const
{
    return Dot(rkU.Cross(rkV));
}
//----------------------------------------------------------------------------
