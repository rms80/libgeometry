// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

//----------------------------------------------------------------------------
inline EdgeKey::EdgeKey (int iV0, int iV1)
{
    if (iV0 < iV1)
    {
        // v0 is minimum
        V[0] = iV0;
        V[1] = iV1;
    }
    else
    {
        // v1 is minimum
        V[0] = iV1;
        V[1] = iV0;
    }
}
//----------------------------------------------------------------------------
inline bool EdgeKey::operator< (const EdgeKey& rkKey) const
{
    if (V[1] < rkKey.V[1])
    {
        return true;
    }

    if (V[1] > rkKey.V[1])
    {
        return false;
    }

    return V[0] < rkKey.V[0];
}
//----------------------------------------------------------------------------
inline EdgeKey::operator size_t () const
{
    return V[0] | (V[1] << 16);
}
//----------------------------------------------------------------------------
