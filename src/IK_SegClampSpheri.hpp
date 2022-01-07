#pragma once
#include "IK_SegClampBase.hpp"
template <typename TSegmentSO3>
class TIK_SegClampSpheri : public TIK_SegClampBase<TSegmentSO3>
{
private:
	virtual bool ClampST(bool clamped[3], Eigen::Quaternionr& rotq) override
	{
		memset(clamped, false, sizeof(bool)*3);
		bool exist_a_lim = (TSegmentSO3::m_limited[TSegmentSO3::R_theta]
						||  TSegmentSO3::m_limited[TSegmentSO3::R_tau]
						||  TSegmentSO3::m_limited[TSegmentSO3::R_phi]);
		if (!exist_a_lim)
			return false;

		// const Eigen::Quaternionr& rotq0 = rotq;
		Real w = rotq.w();
		Real x = rotq.x();
		Real y = rotq.y();
		Real z = rotq.z();
		Real w_s = sqrt(w*w + y*y), x_s, z_s;	//swing: [w_s, x_s, 0, z_s]
		Real w_t, y_t; 								//twist: [w_t, 0, y_t, 0]
		if (abs(w_s) > c_epsilon)
		{
			w_t = w/w_s; y_t = y/w_s;
			x_s = (w*x + y*z)/w_s;
			z_s = (w*z - x*y)/w_s;
		}
		else
		{
			w_t = 1; y_t = 0;
			x_s = x; z_s = z;
		}

		Real theta = atan2(w_s, x_s);
		Real phi = acos(z_s);
		Real theta_clamp = std::max(TSegmentSO3::m_limRange[TSegmentSO3::R_theta][0]
									, std::min(TSegmentSO3::m_limRange[TSegmentSO3::R_theta][1]
											, theta));
		Real phi_clamp = std::max(TSegmentSO3::m_limRange[TSegmentSO3::R_phi][0]
									, std::min(TSegmentSO3::m_limRange[TSegmentSO3::R_phi][1]
											, phi));

		Real sin_half_tau_clamp = y_t;
		Real cos_half_tau_clamp = w_t;

		bool clampedST[3] = {
							theta_clamp != theta,
							TIK_SegClampBase<TSegmentSO3>::ClampT(y_t, w_t, sin_half_tau_clamp, cos_half_tau_clamp),
							phi_clamp != phi
						};
		

		bool clamped_on_swing = (clampedST[TSegmentSO3::R_theta] || clampedST[TSegmentSO3::R_phi]);
		bool clamped_on_twist = (clampedST[TSegmentSO3::R_tau]);
		bool exists_a_clamped = (clamped_on_swing
								|| clamped_on_twist);
		if (!exists_a_clamped)
			return false;
		else
		{
			Eigen::Quaternionr s_q(w_s, x_s, 0, z_s);
			Eigen::Quaternionr t_q(w_t, 0, y_t, 0);
			if (clamped_on_swing)
			{
				s_q.w() = sin(phi_clamp)*sin(theta_clamp);
				s_q.x() = sin(phi_clamp)*cos(theta_clamp);
				s_q.z() = cos(phi_clamp);
				clamped[0] = true;
				clamped[2] = true;
			}
			if (clamped_on_twist)
			{
				t_q.w() = cos_half_tau_clamp;
				t_q.y() = sin_half_tau_clamp;
				clamped[1] = true;
			}
			rotq = s_q * t_q;
			return true;
		}
	}
};