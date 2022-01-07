template <typename TSegmentSO3>
class TIK_SegClampDirect : public TSegmentSO3
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

		Real w = rotq.w();
		Real x = rotq.x();
		Real y = rotq.y();
		Real z = rotq.z();
		Real w_s = sqrt(w*w + y*y), x_s, z_s;	//swing: [w_s, x_s, 0, z_s]
		Real w_t, y_t; 							//twist: [w_t, 0, y_t, 0]
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
		Real theta = 2*asin(x_s);
		Real phi = 2*asin(z_s);

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
				Real x = (Real)sin(v_clamp[TSegmentSO3::R_theta]*0.5);
				Real y = 0;
				Real z = (Real)sin(v_clamp[TSegmentSO3::R_phi]*0.5);
				Real w = sqrt(1-x*x-z*z);
				s_q.w() = w; s_q.x() = x; s_q.y() = y; s_q.z() = z;
				clamped[0] = clampedST[TSegmentSO3::R_theta];
				clamped[2] = clampedST[TSegmentSO3::R_phi];
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