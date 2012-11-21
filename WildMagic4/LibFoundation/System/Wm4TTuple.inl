// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

//----------------------------------------------------------------------------
template <int DIMENSION, class TYPE>
TTuple<DIMENSION,TYPE>::TTuple ()
{
    // Uninitialized for native data.  Initialized for class data as long as
    // TYPE's default constructor initializes its own data.
}
//----------------------------------------------------------------------------
template <int DIMENSION, class TYPE>
TTuple<DIMENSION,TYPE>::~TTuple ()
{
}
//----------------------------------------------------------------------------
template <int DIMENSION, class TYPE>
TTuple<DIMENSION,TYPE>::TTuple (const TTuple& rkT)
{
    for (int i = 0; i < DIMENSION; i++)
    {
        m_atTuple[i] = rkT.m_atTuple[i];
    }
}
//----------------------------------------------------------------------------
template <int DIMENSION, class TYPE>
TTuple<DIMENSION,TYPE>::operator const TYPE* () const
{
    return m_atTuple;
}
//----------------------------------------------------------------------------
template <int DIMENSION, class TYPE>
TTuple<DIMENSION,TYPE>::operator TYPE* ()
{
    return m_atTuple;
}
//----------------------------------------------------------------------------
template <int DIMENSION, class TYPE>
TYPE TTuple<DIMENSION,TYPE>::operator[] (int i) const
{
    assert(0 <= i && i < DIMENSION);
    return m_atTuple[i];
}
//----------------------------------------------------------------------------
template <int DIMENSION, class TYPE>
TYPE& TTuple<DIMENSION,TYPE>::operator[] (int i)
{
    assert(0 <= i && i < DIMENSION);
    return m_atTuple[i];
}
//----------------------------------------------------------------------------
template <int DIMENSION, class TYPE>
TTuple<DIMENSION,TYPE>& TTuple<DIMENSION,TYPE>::operator= (const TTuple& rkT)
{
    for (int i = 0; i < DIMENSION; i++)
    {
        m_atTuple[i] = rkT.m_atTuple[i];
    }
    return *this;
}
//----------------------------------------------------------------------------
template <int DIMENSION, class TYPE>
bool TTuple<DIMENSION,TYPE>::operator== (const TTuple& rkT) const
{
    const size_t uiSize = DIMENSION*sizeof(TYPE);
    return memcmp(m_atTuple,rkT.m_atTuple,uiSize) == 0;
}
//----------------------------------------------------------------------------
template <int DIMENSION, class TYPE>
bool TTuple<DIMENSION,TYPE>::operator!= (const TTuple& rkT) const
{
    const size_t uiSize = DIMENSION*sizeof(TYPE);
    return memcmp(m_atTuple,rkT.m_atTuple,uiSize) != 0;
}
//----------------------------------------------------------------------------
template <int DIMENSION, class TYPE>
bool TTuple<DIMENSION,TYPE>::operator< (const TTuple& rkT) const
{
    const size_t uiSize = DIMENSION*sizeof(TYPE);
    return memcmp(m_atTuple,rkT.m_atTuple,uiSize) < 0;
}
//----------------------------------------------------------------------------
template <int DIMENSION, class TYPE>
bool TTuple<DIMENSION,TYPE>::operator<= (const TTuple& rkT) const
{
    const size_t uiSize = DIMENSION*sizeof(TYPE);
    return memcmp(m_atTuple,rkT.m_atTuple,uiSize) <= 0;
}
//----------------------------------------------------------------------------
template <int DIMENSION, class TYPE>
bool TTuple<DIMENSION,TYPE>::operator> (const TTuple& rkT) const
{
    const size_t uiSize = DIMENSION*sizeof(TYPE);
    return memcmp(m_atTuple,rkT.m_atTuple,uiSize) > 0;
}
//----------------------------------------------------------------------------
template <int DIMENSION, class TYPE>
bool TTuple<DIMENSION,TYPE>::operator>= (const TTuple& rkT) const
{
    const size_t uiSize = DIMENSION*sizeof(TYPE);
    return memcmp(m_atTuple,rkT.m_atTuple,uiSize) >= 0;
}
//----------------------------------------------------------------------------
