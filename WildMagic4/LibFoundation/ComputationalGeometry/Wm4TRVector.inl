// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

//----------------------------------------------------------------------------
template <int VSIZE, int ISIZE>
TRVector<VSIZE,ISIZE>::TRVector ()
{
    // For efficiency in construction of large arrays of vectors, the
    // default constructor does not initialize the vector.
}
//----------------------------------------------------------------------------
template <int VSIZE, int ISIZE>
TRVector<VSIZE,ISIZE>::TRVector (const TRVector& rkV)
{
    for (int i = 0; i < VSIZE; i++)
    {
        m_akTuple[i] = rkV.m_akTuple[i];
    }
}
//----------------------------------------------------------------------------
template <int VSIZE, int ISIZE>
TRVector<VSIZE,ISIZE>::operator const TRational<ISIZE>* () const
{
    return m_akTuple;
}
//----------------------------------------------------------------------------
template <int VSIZE, int ISIZE>
TRVector<VSIZE,ISIZE>::operator TRational<ISIZE>* ()
{
    return m_akTuple;
}
//----------------------------------------------------------------------------
template <int VSIZE, int ISIZE>
TRational<ISIZE> TRVector<VSIZE,ISIZE>::operator[] (int i) const
{
    assert(0 <= i && i < VSIZE);
    return m_akTuple[i];
}
//----------------------------------------------------------------------------
template <int VSIZE, int ISIZE>
TRational<ISIZE>& TRVector<VSIZE,ISIZE>::operator[] (int i)
{
    assert(0 <= i && i < VSIZE);
    return m_akTuple[i];
}
//----------------------------------------------------------------------------
template <int VSIZE, int ISIZE>
TRVector<VSIZE,ISIZE>& TRVector<VSIZE,ISIZE>::operator= (const TRVector& rkV)
{
    for (int i = 0; i < VSIZE; i++)
    {
        m_akTuple[i] = rkV.m_akTuple[i];
    }
    return *this;
}
//----------------------------------------------------------------------------
template <int VSIZE, int ISIZE>
bool TRVector<VSIZE,ISIZE>::operator== (const TRVector& rkV) const
{
    for (int i = 0; i < VSIZE; i++)
    {
        if (m_akTuple[i] != rkV.m_akTuple[i])
        {
            return false;
        }
    }
    return true;
}
//----------------------------------------------------------------------------
template <int VSIZE, int ISIZE>
bool TRVector<VSIZE,ISIZE>::operator!= (const TRVector& rkV) const
{
    return !operator==(rkV);
}
//----------------------------------------------------------------------------
template <int VSIZE, int ISIZE>
int TRVector<VSIZE,ISIZE>::CompareArrays (const TRVector& rkV) const
{
    for (int i = 0; i < VSIZE; i++)
    {
        if (m_akTuple[i] < rkV.m_akTuple[i])
        {
            return -1;
        }
        if (m_akTuple[i] > rkV.m_akTuple[i])
        {
            return +1;
        }
    }
    return 0;
}
//----------------------------------------------------------------------------
template <int VSIZE, int ISIZE>
bool TRVector<VSIZE,ISIZE>::operator< (const TRVector& rkV) const
{
    return CompareArrays(rkV) < 0;
}
//----------------------------------------------------------------------------
template <int VSIZE, int ISIZE>
bool TRVector<VSIZE,ISIZE>::operator<= (const TRVector& rkV) const
{
    return CompareArrays(rkV) <= 0;
}
//----------------------------------------------------------------------------
template <int VSIZE, int ISIZE>
bool TRVector<VSIZE,ISIZE>::operator> (const TRVector& rkV) const
{
    return CompareArrays(rkV) > 0;
}
//----------------------------------------------------------------------------
template <int VSIZE, int ISIZE>
bool TRVector<VSIZE,ISIZE>::operator>= (const TRVector& rkV) const
{
    return CompareArrays(rkV) >= 0;
}
//----------------------------------------------------------------------------
template <int VSIZE, int ISIZE>
TRVector<VSIZE,ISIZE> TRVector<VSIZE,ISIZE>::operator+ (const TRVector& rkV)
    const
{
    TRVector<VSIZE,ISIZE> kSum;
    for (int i = 0; i < VSIZE; i++)
    {
        kSum.m_akTuple[i] = m_akTuple[i] + rkV.m_akTuple[i];
    }
    return kSum;
}
//----------------------------------------------------------------------------
template <int VSIZE, int ISIZE>
TRVector<VSIZE,ISIZE> TRVector<VSIZE,ISIZE>::operator- (const TRVector& rkV)
    const
{
    TRVector<VSIZE,ISIZE> kDiff;
    for (int i = 0; i < VSIZE; i++)
    {
        kDiff.m_akTuple[i] = m_akTuple[i] - rkV.m_akTuple[i];
    }
    return kDiff;
}
//----------------------------------------------------------------------------
template <int VSIZE, int ISIZE>
TRVector<VSIZE,ISIZE> TRVector<VSIZE,ISIZE>::operator*
    (const TRational<ISIZE>& rkR) const
{
    TRVector<VSIZE,ISIZE> kProd;
    for (int i = 0; i < VSIZE; i++)
    {
        kProd.m_akTuple[i] = rkR*m_akTuple[i];
    }
    return kProd;
}
//----------------------------------------------------------------------------
template <int VSIZE, int ISIZE>
TRVector<VSIZE,ISIZE> TRVector<VSIZE,ISIZE>::operator/
    (const TRational<ISIZE>& rkR) const
{
    assert(rkR != 0);

    TRVector<VSIZE,ISIZE> kProd;
    for (int i = 0; i < VSIZE; i++)
    {
        kProd.m_akTuple[i] = m_akTuple[i]/rkR;
    }

    return kProd;
}
//----------------------------------------------------------------------------
template <int VSIZE, int ISIZE>
TRVector<VSIZE,ISIZE> TRVector<VSIZE,ISIZE>::operator- () const
{
    TRVector<VSIZE,ISIZE> kNeg;
    for (int i = 0; i < VSIZE; i++)
    {
        kNeg.m_akTuple[i] = -m_akTuple[i];
    }
    return kNeg;
}
//----------------------------------------------------------------------------
template <int VSIZE, int ISIZE>
TRVector<VSIZE,ISIZE> operator* (const TRational<ISIZE>& rkR,
    const TRVector<VSIZE,ISIZE>& rkV)
{
    TRVector<VSIZE,ISIZE> kProd;
    for (int i = 0; i < VSIZE; i++)
    {
        kProd[i] = rkR*rkV[i];
    }
    return kProd;
}
//----------------------------------------------------------------------------
template <int VSIZE, int ISIZE>
TRVector<VSIZE,ISIZE>& TRVector<VSIZE,ISIZE>::operator+= (const TRVector& rkV)
{
    for (int i = 0; i < VSIZE; i++)
    {
        m_akTuple[i] += rkV.m_akTuple[i];
    }
    return *this;
}
//----------------------------------------------------------------------------
template <int VSIZE, int ISIZE>
TRVector<VSIZE,ISIZE>& TRVector<VSIZE,ISIZE>::operator-= (const TRVector& rkV)
{
    for (int i = 0; i < VSIZE; i++)
    {
        m_akTuple[i] -= rkV.m_akTuple[i];
    }
    return *this;
}
//----------------------------------------------------------------------------
template <int VSIZE, int ISIZE>
TRVector<VSIZE,ISIZE>& TRVector<VSIZE,ISIZE>::operator*=
    (const TRational<ISIZE>& rkR)
{
    for (int i = 0; i < VSIZE; i++)
    {
        m_akTuple[i] *= rkR;
    }
    return *this;
}
//----------------------------------------------------------------------------
template <int VSIZE, int ISIZE>
TRVector<VSIZE,ISIZE>& TRVector<VSIZE,ISIZE>::operator/=
    (const TRational<ISIZE>& rkR)
{
    assert(rkR != 0);
    for (int i = 0; i < VSIZE; i++)
    {
        m_akTuple[i] /= rkR;
    }
    return *this;
}
//----------------------------------------------------------------------------
template <int VSIZE, int ISIZE>
TRational<ISIZE> TRVector<VSIZE,ISIZE>::SquaredLength () const
{
    TRational<ISIZE> kSqrLen = 0;
    for (int i = 0; i < VSIZE; i++)
    {
        kSqrLen += m_akTuple[i]*m_akTuple[i];
    }
    return kSqrLen;
}
//----------------------------------------------------------------------------
template <int VSIZE, int ISIZE>
TRational<ISIZE> TRVector<VSIZE,ISIZE>::Dot (const TRVector& rkV) const
{
    TRational<ISIZE> kDot = 0;
    for (int i = 0; i < VSIZE; i++)
    {
        kDot += m_akTuple[i]*rkV.m_akTuple[i];
    }
    return kDot;
}
//----------------------------------------------------------------------------
