// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

//----------------------------------------------------------------------------
template <class Real>
Query3Filtered<Real>::Query3Filtered (int iVQuantity,
    const Vector3<Real>* akVertex, Real fUncertainty)
    :
    Query3<Real>(iVQuantity,akVertex),
    m_kRQuery(iVQuantity,akVertex)
{
    assert((Real)0.0 <= fUncertainty && fUncertainty <= (Real)1.0);
    m_fUncertainty = fUncertainty;
}
//----------------------------------------------------------------------------
template <class Real>
Query3Filtered<Real>::~Query3Filtered ()
{
}
//----------------------------------------------------------------------------
template <class Real>
Query::Type Query3Filtered<Real>::GetType () const
{
    return Query::QT_FILTERED;
}
//----------------------------------------------------------------------------
template <class Real>
int Query3Filtered<Real>::ToPlane (const Vector3<Real>& rkP, int iV0, int iV1,
    int iV2) const
{
    const Vector3<Real>& rkV0 = m_akVertex[iV0];
    const Vector3<Real>& rkV1 = m_akVertex[iV1];
    const Vector3<Real>& rkV2 = m_akVertex[iV2];

    Real fX0 = rkP[0] - rkV0[0];
    Real fY0 = rkP[1] - rkV0[1];
    Real fZ0 = rkP[2] - rkV0[2];
    Real fX1 = rkV1[0] - rkV0[0];
    Real fY1 = rkV1[1] - rkV0[1];
    Real fZ1 = rkV1[2] - rkV0[2];
    Real fX2 = rkV2[0] - rkV0[0];
    Real fY2 = rkV2[1] - rkV0[1];
    Real fZ2 = rkV2[2] - rkV0[2];

    Real fLen0 = Math<Real>::Sqrt(fX0*fX0 + fY0*fY0 + fZ0*fZ0);
    Real fLen1 = Math<Real>::Sqrt(fX1*fX1 + fY1*fY1 + fZ1*fZ1);
    Real fLen2 = Math<Real>::Sqrt(fX2*fX2 + fY2*fY2 + fZ2*fZ2);
    Real fScaledUncertainty = m_fUncertainty*fLen0*fLen1*fLen2;

    Real fDet3 = Det3(fX0,fY0,fZ0,fX1,fY1,fZ1,fX2,fY2,fZ2);
    if (Math<Real>::FAbs(fDet3) >= fScaledUncertainty)
    {
        return (fDet3 > (Real)0.0 ? +1 : (fDet3 < (Real)0.0 ? -1 : 0));
    }

    return m_kRQuery.ToPlane(rkP,iV0,iV1,iV2);
}
//----------------------------------------------------------------------------
template <class Real>
int Query3Filtered<Real>::ToCircumsphere (const Vector3<Real>& rkP, int iV0,
    int iV1, int iV2, int iV3) const
{
    const Vector3<Real>& rkV0 = m_akVertex[iV0];
    const Vector3<Real>& rkV1 = m_akVertex[iV1];
    const Vector3<Real>& rkV2 = m_akVertex[iV2];
    const Vector3<Real>& rkV3 = m_akVertex[iV3];

    Real fS0x = rkV0[0] + rkP[0];
    Real fD0x = rkV0[0] - rkP[0];
    Real fS0y = rkV0[1] + rkP[1];
    Real fD0y = rkV0[1] - rkP[1];
    Real fS0z = rkV0[2] + rkP[2];
    Real fD0z = rkV0[2] - rkP[2];
    Real fS1x = rkV1[0] + rkP[0];
    Real fD1x = rkV1[0] - rkP[0];
    Real fS1y = rkV1[1] + rkP[1];
    Real fD1y = rkV1[1] - rkP[1];
    Real fS1z = rkV1[2] + rkP[2];
    Real fD1z = rkV1[2] - rkP[2];
    Real fS2x = rkV2[0] + rkP[0];
    Real fD2x = rkV2[0] - rkP[0];
    Real fS2y = rkV2[1] + rkP[1];
    Real fD2y = rkV2[1] - rkP[1];
    Real fS2z = rkV2[2] + rkP[2];
    Real fD2z = rkV2[2] - rkP[2];
    Real fS3x = rkV3[0] + rkP[0];
    Real fD3x = rkV3[0] - rkP[0];
    Real fS3y = rkV3[1] + rkP[1];
    Real fD3y = rkV3[1] - rkP[1];
    Real fS3z = rkV3[2] + rkP[2];
    Real fD3z = rkV3[2] - rkP[2];
    Real fW0 = fS0x*fD0x + fS0y*fD0y + fS0z*fD0z;
    Real fW1 = fS1x*fD1x + fS1y*fD1y + fS1z*fD1z;
    Real fW2 = fS2x*fD2x + fS2y*fD2y + fS2z*fD2z;
    Real fW3 = fS3x*fD3x + fS3y*fD3y + fS3z*fD3z;

    Real fLen0 = Math<Real>::Sqrt(fD0x*fD0x+fD0y*fD0y+fD0z*fD0z+fW0*fW0);
    Real fLen1 = Math<Real>::Sqrt(fD1x*fD1x+fD1y*fD1y+fD1z*fD1z+fW1*fW1);
    Real fLen2 = Math<Real>::Sqrt(fD2x*fD2x+fD2y*fD2y+fD2z*fD2z+fW2*fW2);
    Real fLen3 = Math<Real>::Sqrt(fD3x*fD3x+fD3y*fD3y+fD3z*fD3z+fW3*fW3);
    Real fScaledUncertainty = m_fUncertainty*fLen0*fLen1*fLen2*fLen3;

    Real fDet4 = Det4(fD0x,fD0y,fD0z,fW0,fD1x,fD1y,fD1z,fW1,fD2x,
        fD2y,fD2z,fW2,fD3x,fD3y,fD3z,fW3);

    if (Math<Real>::FAbs(fDet4) >= fScaledUncertainty)
    {
        return (fDet4 > (Real)0.0 ? 1 : (fDet4 < (Real)0.0 ? -1 : 0));
    }

    return m_kRQuery.ToCircumsphere(rkP,iV0,iV1,iV2,iV3);
}
//----------------------------------------------------------------------------
