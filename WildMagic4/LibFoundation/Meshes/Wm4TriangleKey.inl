// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

//----------------------------------------------------------------------------
inline TriangleKey::TriangleKey (int iV0, int iV1, int iV2)
{
    if (iV0 < iV1)
    {
        if (iV0 < iV2)
        {
            // v0 is minimum
            V[0] = iV0;
            V[1] = iV1;
            V[2] = iV2;
        }
        else
        {
            // v2 is minimum
            V[0] = iV2;
            V[1] = iV0;
            V[2] = iV1;
        }
    }
    else
    {
        if (iV1 < iV2)
        {
            // v1 is minimum
            V[0] = iV1;
            V[1] = iV2;
            V[2] = iV0;
        }
        else
        {
            // v2 is minimum
            V[0] = iV2;
            V[1] = iV0;
            V[2] = iV1;
        }
    }
}
//----------------------------------------------------------------------------
inline bool TriangleKey::operator< (const TriangleKey& rkKey) const
{
    if (V[2] < rkKey.V[2])
    {
        return true;
    }

    if (V[2] > rkKey.V[2])
    {
        return false;
    }

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
inline TriangleKey::operator size_t () const
{
    return V[0] | (V[1] << 10) | (V[2] << 20);
}
//----------------------------------------------------------------------------
