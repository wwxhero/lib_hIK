#pragma once

template <typename TSegmentSO3>
class TIK_SegClampDirect : public TSegmentSO3
{
private:
	bool ClampT(const Real &half_s, const Real& half_c
				, Real& half_s_clamp, Real& half_c_clamp)
	{
		half_s_clamp = std::max(m_sin_half_tau_min
						, std::min(m_sin_half_tau_max,
									half_s));
		if (half_s < 0)
			half_c_clamp = std::max(m_cos_half_tau_min, half_c);
		else
			half_c_clamp = std::max(m_cos_half_tau_max, half_c);
		return half_s_clamp != half_s
			|| half_c_clamp != half_c;		
	}

	virtual bool Initialize(CArtiBodyNode* from, CArtiBodyNode* to) override
	{
		if (!TSegmentSO3::Initialize(from, to))
			return false;

		m_sin_half_tau_min = (Real)sin((Real)0.5 * TSegmentSO3::m_limRange[TSegmentSO3::R_tau][0]);
		m_cos_half_tau_min = (Real)cos((Real)0.5 * TSegmentSO3::m_limRange[TSegmentSO3::R_tau][0]);
		m_sin_half_tau_max = (Real)sin((Real)0.5 * TSegmentSO3::m_limRange[TSegmentSO3::R_tau][1]);
		m_cos_half_tau_max = (Real)cos((Real)0.5 * TSegmentSO3::m_limRange[TSegmentSO3::R_tau][1]);

		m_sin_half_theta_min = (Real)sin((Real)0.5 * TSegmentSO3::m_limRange[TSegmentSO3::R_theta][0]);
		m_sin_half_theta_max = (Real)sin((Real)0.5 * TSegmentSO3::m_limRange[TSegmentSO3::R_theta][1]);

		m_sin_half_phi_min = (Real)sin((Real)0.5 * TSegmentSO3::m_limRange[TSegmentSO3::R_phi][0]);
		m_sin_half_phi_max = (Real)sin((Real)0.5 * TSegmentSO3::m_limRange[TSegmentSO3::R_phi][1]);
		return true;
	}

	virtual bool ClampST(bool clamped[3], Eigen::Quaternionr& rotq) override
	{
		memset(clamped, false, sizeof(bool) * 3);
		bool exist_a_lim = (TSegmentSO3::m_limited[TSegmentSO3::R_theta]
			|| TSegmentSO3::m_limited[TSegmentSO3::R_tau]
			|| TSegmentSO3::m_limited[TSegmentSO3::R_phi]);
		if (!exist_a_lim)
			return false;

		Real w = rotq.w();
		Real x = rotq.x();
		Real y = rotq.y();
		Real z = rotq.z();
		Real w_s = sqrt(w*w + y * y), x_s, z_s;	//swing: [w_s, x_s, 0, z_s]
		Real w_t, y_t; 							//twist: [w_t, 0, y_t, 0]
		if (abs(w_s) > c_epsilon)
		{
			w_t = w / w_s; y_t = y / w_s;
			x_s = (w*x + y * z) / w_s;
			z_s = (w*z - x * y) / w_s;
		}
		else
		{
			w_t = 1; y_t = 0;
			x_s = x; z_s = z;
		}

		Real sin_half_theta = x_s;
		Real sin_half_phi = z_s;
		Real sin_half_theta_clamp = std::max(m_sin_half_theta_min
											, std::min(m_sin_half_theta_max
														, sin_half_theta));

		Real sin_half_phi_clamp = std::max(m_sin_half_phi_min
											, std::min(m_sin_half_phi_max
														, sin_half_phi));

		Real sin_half_tau_clamp = y_t;
		Real cos_half_tau_clamp = w_t;

		bool clampedST[3] = {
						sin_half_theta != sin_half_theta_clamp,
						ClampT(y_t, w_t, sin_half_tau_clamp, cos_half_tau_clamp),
						sin_half_phi != sin_half_phi_clamp
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
				Real x = sin_half_theta_clamp;
				Real y = 0;
				Real z = sin_half_phi_clamp;
				Real w = sqrt(1 - x * x - z * z);
				s_q.w() = w; s_q.x() = x; s_q.y() = y; s_q.z() = z;
				clamped[0] = clampedST[TSegmentSO3::R_theta];
				clamped[2] = clampedST[TSegmentSO3::R_phi];
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
private:
	Real m_sin_half_theta_min, m_sin_half_theta_max;
	Real m_sin_half_phi_min, m_sin_half_phi_max;

	Real m_sin_half_tau_min, m_sin_half_tau_max;
	Real m_cos_half_tau_min, m_cos_half_tau_max;
};