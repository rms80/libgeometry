// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#pragma once

#include "config.h"
#include <Frame.h>
#include <Wm4Matrix4.h>
#include <Wm4Quaternion.h>

namespace rms {

class MatrixBlender
{
public:
	enum CombinerMode {
		AverageMatrix,
		AverageQuaternion,
		AverageSO3,
		AverageMatrixExp
	};

	MatrixBlender(CombinerMode eMode) { m_eMode = eMode; }
	~MatrixBlender(void) {}

	CombinerMode & Mode() { return m_eMode; }
	const CombinerMode & Mode() const { return m_eMode; } 

	void Reset();

	void AppendMatrix( float fWeight, const Wml::Matrix3f & vMatrix );

	Wml::Matrix3f Result();

protected:
	CombinerMode m_eMode;
	
	float m_fWeightSum;

	Wml::Matrix3f m_vAvgMatrix;
	Wml::Matrix4f m_vAvgExpMatrix;

	Wml::Quaternionf m_vOrientationQuaternion;
	Wml::Quaternionf m_vAvgQuaternion;
	Wml::Vector3f m_vAvgSO3;

	static Wml::Quaternionf Mat2Quat( const Wml::Matrix3f & mat );
	static Wml::Matrix3f Quat2Mat( const Wml::Quaternionf & quat );

};



}   // end namespace rms