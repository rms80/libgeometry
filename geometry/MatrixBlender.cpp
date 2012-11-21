// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#include <Wm4Math.h>
#include "MatrixBlender.h"
#include "rmsdebug.h"


using namespace rms;





// ports of Cindy's matrix exponential code
//   ( untested - not sure if it actually works... )

template<class Real>
Wml::Matrix4<Real> Inverse( const Wml::Matrix4<Real> & mat, bool & bSuccess ) 
{
	return mat.Inverse();

 //   Wml::Matrix4<Real> mo;
 //   const Wml::Matrix4<Real> & mi = mat;
 //   
 //   Real t[4][8], save;
 //   int i,j;
 //   // copy mi into t
 //   // set t's last four rows to the identity matrix
 //   for (i=0;i<4;i++)
 //       for (j=0;j<4;j++) {
 //           t[i][j]=mi(i, j);
 //           t[i][j+4]=0;
 //       }
 //   for (i=0;i<4;i++)
 //       t[i][i+4]=1;
 //   
	//mo = Wml::Matrix4<Real>::IDENTITY;
 //   for (i=0;i<4;i++) {
 //       if (i<3) {	// swap row with largest front coefficient
 //           Real a=fabs(t[i][i]),ab;
 //           int m=i;
 //           for (int l=i+1;l<4;l++) {
 //               if ((ab=fabs(t[l][i]))>a) {
 //                   a=ab,m=l;
 //               }
 //           }
 //           if (m!=i) {
 //               for (j=0;j<8;j++) {
 //                   save = t[i][j];
 //                   t[i][j] = t[m][j];
 //                   t[m][j] = save;
 //               }
 //           }
 //       }
 //       if (!t[i][i]) {
 //           bSuccess = false ;
 //           return mo;
 //       }
 //       for (j=0;j<4;j++) {
	//		if (i==j) {
 //                  Real a=1/t[i][i];
 //                  for (int k=0;k<8;k++)
 //                      t[j][k]*=a;
	//		} else {
 //                  Real a=-t[j][i]/t[i][i];
 //                  for (int k=0;k<8;k++)
 //                      t[j][k]+=a*t[i][k];
	//		}
 //       }
 //   }
 //   for (i=0;i<4;i++)
 //       for (j=0;j<4;j++)
 //           mo(i,j)=t[i][j+4];
 //   
 //   bSuccess = TRUE ;
 //   return mo;
}




// returns the Frobenius norm
template<class Real>
double FrobeniusNorm(const Wml::Matrix4<Real> & vMatrix)
{
   double out = 0;

   for (int i = 0; i < 4; i += 1)
        for (int j = 0; j < 4; j += 1)
            out += pow( vMatrix[i][j], 2);

   return sqrt(out);
}


// raises the matrix to the p power
template<class Real>
Wml::Matrix4<Real> Pow(const Wml::Matrix4<Real> & vMatrix, int p) 
{
	Wml::Matrix4<Real> out_mat = vMatrix;
	Wml::Matrix4<Real> temp = vMatrix;

	// don't deal with negative exponents, just return identity
	if (p <= 0) {
		out_mat = Wml::Matrix4<Real>::IDENTITY;
		return out_mat;
	}

	for (int i = 0; i < p - 1; i++) {
		out_mat = out_mat * temp;
	}
	return out_mat;
}



template<class Real>
Wml::Matrix4<Real> Sqrt( const Wml::Matrix4<Real> & vMatrix ) 
{
   Wml::Matrix4<Real> A = vMatrix;          // call it A to be like Alexa's pseudocode
   Wml::Matrix4<Real> X = vMatrix;
   Wml::Matrix4<Real> Y = Wml::Matrix4<Real>::IDENTITY;;

   bool bSuc = false;
   int i = 0;
   double eps3 = 0.000001;
   int n3 = 10;

   while ( FrobeniusNorm(X*X - A) > eps3) {
      double error = FrobeniusNorm(X*X - A);
      Wml::Matrix4<Real> iX = Inverse(X, bSuc);
      Wml::Matrix4<Real> iY = Inverse(Y, bSuc);
      X = (X + iY)/(Real)2.0;
      Y = (Y + iX)/(Real)2.0;
      i++;
      if (i > n3) { 
         if (error > 0.01)
			 _RMSInfo("sqrt: failed to converge, error = %f\n", error);
         break;
      }
   }
   return X;
}







template<class Real>
Wml::Matrix4<Real> Exp( const Wml::Matrix4<Real> & vMatrix ) 
{
	Wml::Matrix4<Real> A = vMatrix;               // call it A to be like Alexa's pseudocode
    Wml::Matrix4<Real> X = Wml::Matrix4<Real>::IDENTITY;  // the current sum
    Wml::Matrix4<Real> D = Wml::Matrix4<Real>::IDENTITY;  // denominator
    Wml::Matrix4<Real> N = Wml::Matrix4<Real>::IDENTITY;  // numerator
    double c = 1.0;                           // coefficienty thing

	int j = (int) std::max(0.0, 1.0 + floor(log(FrobeniusNorm(A))/log(2.0)));  // gives logbase2(A.Norm())
	A = A * (Real)pow(2.0,-j);

	int q = 6;      // supposedly 6 is a good number of iterations
	for (int k = 1; k <= q; k++) {
		c = c*(q - k + 1.0)/(Real)(k*(2*q - k + 1.0));
		X = A*X;
		N = N + X*(Real)c;
		D = D + X*(Real)(pow(-1.0,k)*c);
	}

	bool bSuc = false;
	X = Inverse(D,bSuc) * N;
	int p = (int)pow(2.0,j);
	X = Pow(X,p);
	return X;
}



template<class Real>
Wml::Matrix4<Real> Log( const Wml::Matrix4<Real> & vMatrix /*int in_n , float &out_id*/ )
{
   Wml::Matrix4<Real> A = vMatrix;           // call it A to be like Alexa's pseudocode
   Wml::Matrix4<Real> I = Wml::Matrix4<Real>::IDENTITY;   // identity matrix
   //A.PrintMatlab();

   int k = 0;
   int n1 = 30;
   double eps1 = 0.0001;
   while ( FrobeniusNorm(A-I) > eps1 /*&& k < in_n*/ ) {
      double error = FrobeniusNorm(A-I);
      A = Sqrt(A);
      k++;

      if (k > n1) {
         printf("log: repeated square roots failed to converge after %d iterations\n", n1);
         break;
      }
   }

   A = A - I;
   Wml::Matrix4<Real> Z = A;
   Wml::Matrix4<Real> X = A;
   Real i = 1.0;
   double eps2 = 0.000000001;
   int n2 = 7;

   while ( FrobeniusNorm(Z) > eps2 ) {
      Z = Z*A;
      i++;
      X = X + Z/i;
      if (i > n2) { 
         printf("log: failed to converge after %d iterations\n", n2);
         break;
      }
   }

   X = (Real)pow(2.0,k) * X;
   return X;
}







// from here: http://cache-www.intel.com/cd/00/00/29/37/293748_293748.pdf
Wml::Quaternionf MatrixBlender::Mat2Quat( const Wml::Matrix3f & mat ) 
{
	float s0, s1, s2;
	int k0, k1, k2, k3;

	float q[4] = {0,0,0,0};
	const float *m = *&mat;
	if ( m[0 * 3 + 0] + m[1 * 3 + 1] + m[2 * 3 + 2] > 0.000001f ) {
		k0 = 3;
		k1 = 2;
		k2 = 1;
		k3 = 0;
		s0 = 1.0f;
		s1 = 1.0f;
		s2 = 1.0f;
	} else if ( m[0 * 3 + 0] > m[1 * 3 + 1] && m[0 * 3 + 0] > m[2 * 3 + 2] ) {
		k0 = 0;
		k1 = 1;
		k2 = 2;
		k3 = 3;
		s0 = 1.0f;
		s1 = -1.0f;
		s2 = -1.0f;
	} else if ( m[1 * 3 + 1] > m[2 * 3 + 2] ) {
		k0 = 1;
		k1 = 0;
		k2 = 3;
		k3 = 2;
		s0 = -1.0f;
		s1 = 1.0f;
		s2 = -1.0f;
	} else {
		k0 = 2;
		k1 = 3;
		k2 = 0;
		k3 = 1;
		s0 = -1.0f;
		s1 = -1.0f;
		s2 = 1.0f;
	}
	float t = s0 * m[0 * 3 + 0] + s1 * m[1 * 3 + 1] + s2 * m[2 * 3 + 2] + 1.0f;
	float s = (1.0f / (float)sqrt( t )) * 0.5f;
	q[k0] = s * t;
	q[k1] = ( m[0 * 3 + 1] - s2 * m[1 * 3 + 0] ) * s;
	q[k2] = ( m[2 * 3 + 0] - s1 * m[0 * 3 + 2] ) * s;
	q[k3] = ( m[1 * 3 + 2] - s0 * m[2 * 3 + 1] ) * s;

	return Wml::Quaternionf(q[3],q[0],q[1],q[2]);
}


// from here: http://cache-www.intel.com/cd/00/00/29/37/293748_293748.pdf
Wml::Matrix3f MatrixBlender::Quat2Mat( const Wml::Quaternionf & quat ) 
{
	const float q[4] = {quat[1],quat[2],quat[3],quat[0]};
	Wml::Matrix3f mat;
	float *m = *&mat;
	float x2 = q[0] + q[0];
	float y2 = q[1] + q[1];
	float z2 = q[2] + q[2];
	float w2 = q[3] + q[3];
	float yy2 = q[1] * y2;
	float xy2 = q[0] * y2;
	float xz2 = q[0] * z2;
	float yz2 = q[1] * z2;
	float zz2 = q[2] * z2;
	float wz2 = q[3] * z2;
	float wy2 = q[3] * y2;
	float wx2 = q[3] * x2;
	float xx2 = q[0] * x2;
	m[0*3+0] = - yy2 - zz2 + 1.0f;
	m[0*3+1] = xy2 + wz2;
	m[0*3+2] = xz2 - wy2;
	m[1*3+0] = xy2 - wz2;
	m[1*3+1] = - xx2 - zz2 + 1.0f;
	m[1*3+2] = yz2 + wx2;
	m[2*3+0] = xz2 + wy2;
	m[2*3+1] = yz2 - wx2;
	m[2*3+2] = - xx2 - yy2 + 1.0f;
	return mat;
}





template <class Real>
Wml::Quaternion<Real> fromExponentialMap(const Wml::Vector3<Real>& v, bool reparam)
{
	Real theta = v.Length();

	if(reparam)
	{
		if(theta > +Wml::Math<Real>::PI) { theta = (theta - Wml::Math<Real>::TWO_PI); }
		if(theta < -Wml::Math<Real>::PI) { theta = (Wml::Math<Real>::TWO_PI + theta); }
	}

	Real halftheta = (Real)0.5 * theta;

	Real c = (theta > Wml::Math<Real>::EPSILON) ? ( (Real)sin(halftheta) / theta ) : ( (Real)0.5 + (theta*theta)/(Real)48.0 );

	float fTuple[4];
	fTuple[0] = (Real)cos(halftheta);
	for(int i=0; i<3; i++) 
	{ 
		fTuple[i+1] = c * v[i]; 
	}
	
	return Wml::Quaternion<Real>(fTuple[0], fTuple[1], fTuple[2], fTuple[3]);
}


template <class Real>
Wml::Vector3<Real> toExponentialMap(const Wml::Quaternion<Real> & quat, bool reparam)
{
	Wml::Vector3<Real> v;
	if((Real)fabs(quat[0]) < (Real)1.0)
	{
		Real halftheta = (Real)acos(quat[0]);
		Real length = (Real)sin(halftheta);

		if((Real)fabs(length) > Wml::Math<Real>::EPSILON)
		{
			Real c = ((Real)2.0 * halftheta) / length;

			if(reparam)
			{
				if(c > +Wml::Math<Real>::PI) { c = (c - Wml::Math<Real>::TWO_PI); }
				if(c < -Wml::Math<Real>::PI) { c = (Wml::Math<Real>::TWO_PI + c); }
			}

			for(int i=0; i<3; i++) { 
				v[i] = c * quat[i+1]; 
			}
			return v;
		}
	}

	for(int i=0; i<3; i++) { 
		v[i] = quat[i+1]; 
	}
	return v;
}



void MatrixBlender::Reset()
{
	m_fWeightSum = 0.0f;

	switch ( m_eMode ) {
		case AverageMatrix:
			m_vAvgMatrix = Wml::Matrix3f::ZERO;
			break;
		case AverageQuaternion:
			m_vAvgQuaternion = Wml::Quaternionf::ZERO;
			break;
		case AverageSO3:
			m_vAvgSO3 = Wml::Vector3f::ZERO;
			break;
		case AverageMatrixExp:
			m_vAvgExpMatrix = Wml::Matrix4f::ZERO;
			break;
	}
}


void MatrixBlender::AppendMatrix( float fWeight, const Wml::Matrix3f & m )
{
	bool bFirst = (m_fWeightSum == 0.0f);

	m_fWeightSum += fWeight;
	switch ( m_eMode ) {
		case AverageMatrix: {
				m_vAvgMatrix += fWeight * m;
			} break;

		case AverageQuaternion: {
				Wml::Quaternionf quat(Mat2Quat(m));
				quat.Normalize();
				if ( bFirst )
					m_vOrientationQuaternion = quat;
				else if ( quat.Dot(m_vOrientationQuaternion) < 0.0f )
					quat = -quat;
				m_vAvgQuaternion += fWeight * quat;
			} break;

		case AverageSO3: {
				Wml::Quaternionf quat(Mat2Quat(m));
				quat.Normalize();
				if ( bFirst )
					m_vOrientationQuaternion = quat;
				else if ( quat.Dot(m_vOrientationQuaternion) < 0.0f )
					quat = -quat;
				m_vAvgSO3 += fWeight * toExponentialMap(quat, false);
			} break;

		case AverageMatrixExp: {
				Wml::Matrix4f mat4( m[0][0], m[0][1], m[0][2], 0.0f, 
									m[1][0], m[1][1], m[1][2], 0.0f, 
									m[2][0], m[2][1], m[2][2], 0.0f, 
									0.0f, 0.0f, 0.0f, 1.0f );
				m_vAvgExpMatrix += Log(mat4) * fWeight;
			} break;
	}
}


Wml::Matrix3f MatrixBlender::Result()
{
	switch ( m_eMode ) {
		case AverageMatrix: {
				return m_vAvgMatrix / m_fWeightSum;
			} break;

		case AverageQuaternion: {
				Wml::Quaternionf result(m_vAvgQuaternion);
				result.Normalize();
				return Quat2Mat(result);
			} break;

		case AverageSO3: {
				Wml::Quaternionf result( fromExponentialMap(m_vAvgSO3, false) );
				result.Normalize();
				return Quat2Mat(result);
			} break;

		case AverageMatrixExp: {
				Wml::Matrix4f m( Exp( m_vAvgExpMatrix / m_fWeightSum ) );
				return Wml::Matrix3f( m[0][0], m[0][1], m[0][2],   m[1][0], m[1][1], m[1][2],   m[2][0], m[2][1], m[2][2] );
			} break;

		default:
			return Wml::Matrix3f::ZERO;
	}
}
