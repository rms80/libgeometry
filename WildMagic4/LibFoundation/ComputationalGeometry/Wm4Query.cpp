// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

#include "Wm4FoundationPCH.h"
#include "Wm4Query.h"
using namespace Wm4;

//----------------------------------------------------------------------------
Query::Query ()
{
}
//----------------------------------------------------------------------------
Query::~Query ()
{
}
//----------------------------------------------------------------------------
bool Query::Sort (int& iV0, int& iV1)
{
    int j0, j1;
    bool bPositive;

    if (iV0 < iV1)
    {
        j0 = 0; j1 = 1; bPositive = true;
    }
    else
    {
        j0 = 1; j1 = 0; bPositive = false;
    }

    int aiValue[2] = { iV0, iV1 };
    iV0 = aiValue[j0];
    iV1 = aiValue[j1];
    return bPositive;
}
//----------------------------------------------------------------------------
bool Query::Sort (int& iV0, int& iV1, int& iV2)
{
    int j0, j1, j2;
    bool bPositive;

    if (iV0 < iV1)
    {
        if (iV2 < iV0)
        {
            j0 = 2; j1 = 0; j2 = 1; bPositive = true;
        }
        else if (iV2 < iV1)
        {
            j0 = 0; j1 = 2; j2 = 1; bPositive = false;
        }
        else
        {
            j0 = 0; j1 = 1; j2 = 2; bPositive = true;
        }
    }
    else
    {
        if (iV2 < iV1)
        {
            j0 = 2; j1 = 1; j2 = 0; bPositive = false;
        }
        else if (iV2 < iV0)
        {
            j0 = 1; j1 = 2; j2 = 0; bPositive = true;
        }
        else
        {
            j0 = 1; j1 = 0; j2 = 2; bPositive = false;
        }
    }

    int aiValue[3] = { iV0, iV1, iV2 };
    iV0 = aiValue[j0];
    iV1 = aiValue[j1];
    iV2 = aiValue[j2];
    return bPositive;
}
//----------------------------------------------------------------------------
bool Query::Sort (int& iV0, int& iV1, int& iV2, int& iV3)
{
    int j0, j1, j2, j3;
    bool bPositive;

    if (iV0 < iV1)
    {
        if (iV2 < iV3)
        {
            if (iV1 < iV2)
            {
                j0 = 0; j1 = 1; j2 = 2; j3 = 3; bPositive = true;
            }
            else if (iV3 < iV0)
            {
                j0 = 2; j1 = 3; j2 = 0; j3 = 1; bPositive = true;
            }
            else if (iV2 < iV0)
            {
                if (iV3 < iV1)
                {
                    j0 = 2; j1 = 0; j2 = 3; j3 = 1; bPositive = false;
                }
                else
                {
                    j0 = 2; j1 = 0; j2 = 1; j3 = 3; bPositive = true;
                }
            }
            else
            {
                if (iV3 < iV1)
                {
                    j0 = 0; j1 = 2; j2 = 3; j3 = 1; bPositive = true;
                }
                else
                {
                    j0 = 0; j1 = 2; j2 = 1; j3 = 3; bPositive = false;
                }
            }
        }
        else
        {
            if (iV1 < iV3)
            {
                j0 = 0; j1 = 1; j2 = 3; j3 = 2; bPositive = false;
            }
            else if (iV2 < iV0)
            {
                j0 = 3; j1 = 2; j2 = 0; j3 = 1; bPositive = false;
            }
            else if (iV3 < iV0)
            {
                if (iV2 < iV1)
                {
                    j0 = 3; j1 = 0; j2 = 2; j3 = 1; bPositive = true;
                }
                else
                {
                    j0 = 3; j1 = 0; j2 = 1; j3 = 2; bPositive = false;
                }
            }
            else
            {
                if (iV2 < iV1)
                {
                    j0 = 0; j1 = 3; j2 = 2; j3 = 1; bPositive = false;
                }
                else
                {
                    j0 = 0; j1 = 3; j2 = 1; j3 = 2; bPositive = true;
                }
            }
        }
    }
    else
    {
        if (iV2 < iV3)
        {
            if (iV0 < iV2)
            {
                j0 = 1; j1 = 0; j2 = 2; j3 = 3; bPositive = false;
            }
            else if (iV3 < iV1)
            {
                j0 = 2; j1 = 3; j2 = 1; j3 = 0; bPositive = false;
            }
            else if (iV2 < iV1)
            {
                if (iV3 < iV0)
                {
                    j0 = 2; j1 = 1; j2 = 3; j3 = 0; bPositive = true;
                }
                else
                {
                    j0 = 2; j1 = 1; j2 = 0; j3 = 3; bPositive = false;
                }
            }
            else
            {
                if (iV3 < iV0)
                {
                    j0 = 1; j1 = 2; j2 = 3; j3 = 0; bPositive = false;
                }
                else
                {
                    j0 = 1; j1 = 2; j2 = 0; j3 = 3; bPositive = true;
                }
            }
        }
        else
        {
            if (iV0 < iV3)
            {
                j0 = 1; j1 = 0; j2 = 3; j3 = 2; bPositive = true;
            }
            else if (iV2 < iV1)
            {
                j0 = 3; j1 = 2; j2 = 1; j3 = 0; bPositive = true;
            }
            else if (iV3 < iV1)
            {
                if (iV2 < iV0)
                {
                    j0 = 3; j1 = 1; j2 = 2; j3 = 0; bPositive = false;
                }
                else
                {
                    j0 = 3; j1 = 1; j2 = 0; j3 = 2; bPositive = true;
                }
            }
            else
            {
                if (iV2 < iV0)
                {
                    j0 = 1; j1 = 3; j2 = 2; j3 = 0; bPositive = true;
                }
                else
                {
                    j0 = 1; j1 = 3; j2 = 0; j3 = 2; bPositive = false;
                }
            }
        }
    }

    int aiValue[4] = { iV0, iV1, iV2, iV3 };
    iV0 = aiValue[j0];
    iV1 = aiValue[j1];
    iV2 = aiValue[j2];
    iV3 = aiValue[j3];
    return bPositive;
}
//----------------------------------------------------------------------------
