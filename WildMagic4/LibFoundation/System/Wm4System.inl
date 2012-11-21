// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

//----------------------------------------------------------------------------
template <class T>
void Allocate (int iCols, int iRows, T**& raatArray)
{
    raatArray = WM4_NEW T*[iRows];
    raatArray[0] = WM4_NEW T[iRows*iCols];
    for (int iRow = 1; iRow < iRows; iRow++)
    {
        raatArray[iRow] = &raatArray[0][iCols*iRow];
    }
}
//----------------------------------------------------------------------------
template <class T>
void Deallocate (T**& raatArray)
{
    if (raatArray)
    {
        WM4_DELETE[] raatArray[0];
        WM4_DELETE[] raatArray;
        raatArray = 0;
    }
}
//----------------------------------------------------------------------------
template <class T>
void Allocate (int iCols, int iRows, int iSlices, T***& raaatArray)
{
    raaatArray = WM4_NEW T**[iSlices];
    raaatArray[0] = WM4_NEW T*[iSlices*iRows];
    raaatArray[0][0] = WM4_NEW T[iSlices*iRows*iCols];
    for (int iSlice = 0; iSlice < iSlices; iSlice++)
    {
        raaatArray[iSlice] = &raaatArray[0][iRows*iSlice];
        for (int iRow = 0; iRow < iRows; iRow++)
        {
            raaatArray[iSlice][iRow] =
                &raaatArray[0][0][iCols*(iRow+iRows*iSlice)];
        }
    }
}
//----------------------------------------------------------------------------
template <class T>
void Deallocate (T***& raaatArray)
{
    if (raaatArray)
    {
        WM4_DELETE[] raaatArray[0][0];
        WM4_DELETE[] raaatArray[0];
        WM4_DELETE[] raaatArray;
        raaatArray = 0;
    }
}
//----------------------------------------------------------------------------
