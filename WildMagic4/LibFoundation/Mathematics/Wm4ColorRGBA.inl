// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

//----------------------------------------------------------------------------
inline ColorRGBA::operator const float* () const
{
    return m_afTuple;
}
//----------------------------------------------------------------------------
inline ColorRGBA::operator float* ()
{
    return m_afTuple;
}
//----------------------------------------------------------------------------
inline float ColorRGBA::operator[] (int i) const
{
    assert(0 <= i && i <= 3);
    if (i < 0)
    {
        i = 0;
    }
    else if ( i > 3 )
    {
        i = 3;
    }

    return m_afTuple[i];
}
//----------------------------------------------------------------------------
inline float& ColorRGBA::operator[] (int i)
{
    assert(0 <= i && i <= 3);
    if (i < 0)
    {
        i = 0;
    }
    else if ( i > 3 )
    {
        i = 3;
    }

    return m_afTuple[i];
}
//----------------------------------------------------------------------------
inline float ColorRGBA::R () const
{
    return m_afTuple[0];
}
//----------------------------------------------------------------------------
inline float& ColorRGBA::R ()
{
    return m_afTuple[0];
}
//----------------------------------------------------------------------------
inline float ColorRGBA::G () const
{
    return m_afTuple[1];
}
//----------------------------------------------------------------------------
inline float& ColorRGBA::G ()
{
    return m_afTuple[1];
}
//----------------------------------------------------------------------------
inline float ColorRGBA::B () const
{
    return m_afTuple[2];
}
//----------------------------------------------------------------------------
inline float& ColorRGBA::B ()
{
    return m_afTuple[2];
}
//----------------------------------------------------------------------------
inline float ColorRGBA::A () const
{
    return m_afTuple[3];
}
//----------------------------------------------------------------------------
inline float& ColorRGBA::A ()
{
    return m_afTuple[3];
}
//----------------------------------------------------------------------------
