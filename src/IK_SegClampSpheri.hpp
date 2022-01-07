template <typename TSegmentSO3>
class TIK_SegClampSpheri : public TSegmentSO3
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

		Real tau = wrap_pi(2*atan2(y_t, w_t));
		Real theta = atan2(w_s, x_s);
		Real phi = acos(z_s);

		Real v[3] = {theta, tau, phi};

		Real v_clamp[3] = {0};
		bool clampedST[3] = {false};
		for (int i_dof = 0; i_dof < TSegmentSO3::c_num_DoFs; i_dof ++)
		{
			v_clamp[i_dof] = std::max(TSegmentSO3::m_limRange[i_dof][0], std::min(TSegmentSO3::m_limRange[i_dof][1], v[i_dof]));
			clampedST[i_dof] = (v_clamp[i_dof] != v[i_dof]);
		}

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
				s_q.w() = sin(v_clamp[TSegmentSO3::R_phi])*sin(v_clamp[TSegmentSO3::R_theta]);
				s_q.x() = sin(v_clamp[TSegmentSO3::R_phi])*cos(v_clamp[TSegmentSO3::R_theta]);
				s_q.z() = cos(v_clamp[TSegmentSO3::R_phi]);
				clamped[0] = true;
				clamped[2] = true;
			}
			if (clamped_on_twist)
			{
				Real tau_half = v_clamp[TSegmentSO3::R_tau]*(Real)0.5;
				t_q.w() = cos(tau_half);
				t_q.y() = sin(tau_half);
				clamped[1] = true;
			}
			rotq = s_q * t_q;
			return true;
		}
	}
};