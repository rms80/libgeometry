// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

//----------------------------------------------------------------------------
inline size_t& Memory::MaxAllowedBytes ()
{
    return ms_uiMaxAllowedBytes;
}
//----------------------------------------------------------------------------
inline bool& Memory::TrackSizes ()
{
    return ms_bTrackSizes;
}
//----------------------------------------------------------------------------
inline size_t Memory::GetNumNewCalls ()
{
    return ms_uiNumNewCalls;
}
//----------------------------------------------------------------------------
inline size_t Memory::GetNumDeleteCalls ()
{
    return ms_uiNumDeleteCalls;
}
//----------------------------------------------------------------------------
inline size_t Memory::GetNumBlocks ()
{
    return ms_uiNumBlocks;
}
//----------------------------------------------------------------------------
inline size_t Memory::GetNumBytes ()
{
    return ms_uiNumBytes;
}
//----------------------------------------------------------------------------
inline size_t Memory::GetMaxAllocatedBytes ()
{
    return ms_uiMaxAllocatedBytes;
}
//----------------------------------------------------------------------------
inline size_t Memory::GetMaxBlockSize ()
{
    return ms_uiMaxBlockSize;
}
//----------------------------------------------------------------------------
inline size_t Memory::GetHistogram (int i)
{
    if (0 <= i && i <= 31)
    {
        return ms_auiHistogram[i];
    }

    return 0;
}
//----------------------------------------------------------------------------
inline const Memory::Block* Memory::GetHead ()
{
    return ms_pkHead;
}
//----------------------------------------------------------------------------
inline const Memory::Block* Memory::GetTail ()
{
    return ms_pkTail;
}
//----------------------------------------------------------------------------
