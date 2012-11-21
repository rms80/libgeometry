// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 4.10.0 (2009/11/18)

//----------------------------------------------------------------------------
template <class Real>
Tetrahedron3<Real>::Tetrahedron3 ()
{
    // uninitialized
}
//----------------------------------------------------------------------------
template <class Real>
Tetrahedron3<Real>::Tetrahedron3 (const Vector3<Real>& rkV0,
    const Vector3<Real>& rkV1, const Vector3<Real>& rkV2,
    const Vector3<Real>& rkV3)
{
    V[0] = rkV0;
    V[1] = rkV1;
    V[2] = rkV2;
    V[3] = rkV3;
}
//----------------------------------------------------------------------------
template <class Real>
Tetrahedron3<Real>::Tetrahedron3 (const Vector3<Real> akV[4])
{
    V[0] = akV[0];
    V[1] = akV[1];
    V[2] = akV[2];
    V[3] = akV[3];
}
//----------------------------------------------------------------------------
template <class Real>
void Tetrahedron3<Real>::GetFaceIndices (int iFace, int aiIndex[3]) const
{
    switch (iFace)
    {
    case 0:
        aiIndex[0] = 0;  aiIndex[1] = 2;  aiIndex[2] = 1;
        break;
    case 1:
        aiIndex[0] = 0;  aiIndex[1] = 1;  aiIndex[2] = 3;
        break;
    case 2:
        aiIndex[0] = 0;  aiIndex[1] = 3;  aiIndex[2] = 2;
        break;
    case 3:
        aiIndex[0] = 1;  aiIndex[1] = 2;  aiIndex[2] = 3;
        break;
    default:
        assert( false );
        break;
    }
}
//----------------------------------------------------------------------------
template <class Real>
void Tetrahedron3<Real>::GetPlanes (Plane3<Real> akPlane[4],
    bool bUnitLengthNormals) const
{
    Vector3<Real> kEdge10 = V[1] - V[0];
    Vector3<Real> kEdge20 = V[2] - V[0];
    Vector3<Real> kEdge30 = V[3] - V[0];
    Vector3<Real> kEdge21 = V[2] - V[1];
    Vector3<Real> kEdge31 = V[3] - V[1];

    if (bUnitLengthNormals)
    {
        akPlane[0].Normal = kEdge20.UnitCross(kEdge10);  // <v0,v2,v1>
        akPlane[1].Normal = kEdge10.UnitCross(kEdge30);  // <v0,v1,v3>
        akPlane[2].Normal = kEdge30.UnitCross(kEdge20);  // <v0,v3,v2>
        akPlane[3].Normal = kEdge21.UnitCross(kEdge31);  // <v1,v2,v3>
    }
    else
    {
        akPlane[0].Normal = kEdge20.Cross(kEdge10);  // <v0,v2,v1>
        akPlane[1].Normal = kEdge10.Cross(kEdge30);  // <v0,v1,v3>
        akPlane[2].Normal = kEdge30.Cross(kEdge20);  // <v0,v3,v2>
        akPlane[3].Normal = kEdge21.Cross(kEdge31);  // <v1,v2,v3>
    }

    Real fDet = kEdge10.Dot(akPlane[3].Normal);
    int i;
    if (fDet < (Real)0.0)
    {
        // normals are inner pointing, reverse their directions
        for (i = 0; i < 4; i++)
        {
            akPlane[i].Normal = -akPlane[i].Normal;
        }
    }

    for (i = 0; i < 4; i++)
    {
        akPlane[i].Constant = V[i].Dot(akPlane[i].Normal);
    }
}
//----------------------------------------------------------------------------
