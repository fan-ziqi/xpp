/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <xpp_cyberdog/cyberdogleg_inverse_kinematics.h>

#include <cmath>
#include <map>

#include <xpp_states/cartesian_declarations.h>


namespace xpp
{
	CyberdoglegInverseKinematics::Vector3d
	CyberdoglegInverseKinematics::GetJointAngles(const Vector3d &ee_pos_H, int _sideSign) const
	{
		Vector3d pEe2H;
		pEe2H = ee_pos_H;

		double q1, q2, q3;
		Vector3d qResult;
		double px, py, pz;
		double b2y, b3z, b4z, a, b, c;

		px = pEe2H(0);
		py = pEe2H(1);
		pz = pEe2H(2);

		b2y = _abadLinkLength * _sideSign;
		b3z = -_hipLinkLength;
		b4z = -_kneeLinkLength;
		a = _abadLinkLength;
		c = sqrt(pow(px, 2) + pow(py, 2) + pow(pz, 2)); // whole length
		b = sqrt(pow(c, 2) - pow(a, 2)); // distance between shoulder and footpoint

		q1 = q1_ik(py, pz, b2y);
		q3 = q3_ik(b3z, b4z, b);
		q2 = q2_ik(q1, q3, px, py, pz, b3z, b4z);

		EnforceLimits(q1, HAA);
		EnforceLimits(q2, HFE);
		EnforceLimits(q3, KFE);

		qResult(0) = q1;
		qResult(1) = q2;
		qResult(2) = q3;

		return qResult;
	}

	double CyberdoglegInverseKinematics::q1_ik(double py, double pz, double l1)
	{
		float q1;
		float L = sqrt(pow(py, 2) + pow(pz, 2) - pow(l1, 2));
		q1 = atan2(pz * l1 + py * L, py * l1 - pz * L);
		return -q1;
	}

	double CyberdoglegInverseKinematics::q3_ik(double b3z, double b4z, double b)
	{
		double q3, temp;
		temp = (pow(b3z, 2) + pow(b4z, 2) - pow(b, 2)) / (2 * fabs(b3z * b4z));
		if(temp > 1) temp = 1;
		if(temp < -1) temp = -1;
		q3 = acos(temp);
		q3 = -(M_PI - q3); //0~180
		return q3;
	}

	double CyberdoglegInverseKinematics::q2_ik(double q1, double q3, double px, double py, double pz, double b3z, double b4z)
	{
		double q2, a1, a2, m1, m2;

		a1 = py * sin(q1) - pz * cos(q1);
		a2 = px;
		m1 = b4z * sin(q3);
		m2 = b3z + b4z * cos(q3);
		q2 = atan2(m1 * a1 + m2 * a2, m1 * a2 - m2 * a1);
		return q2;
	}

	void
	CyberdoglegInverseKinematics::EnforceLimits(double &val, CyberdogJointID joint) const
	{
		// totally exaggerated joint angle limits
		const static double haa_min = -0.75;
		const static double haa_max = 0.75;

		const static double hfe_min = -1.257;
		const static double hfe_max = 3.49;

		const static double kfe_min = -2.478;
		const static double kfe_max = -0.506;

		// reduced joint angles for optimization
		static const std::map<CyberdogJointID, double> max_range{
				{HAA, haa_max},
				{HFE, hfe_max},
				{KFE, kfe_max}
		};

		// reduced joint angles for optimization
		static const std::map<CyberdogJointID, double> min_range{
				{HAA, haa_min},
				{HFE, hfe_min},
				{KFE, kfe_min}
		};

		double max = max_range.at(joint);
		val = val > max ? max : val;

		double min = min_range.at(joint);
		val = val < min ? min : val;
	}

} /* namespace xpp */
