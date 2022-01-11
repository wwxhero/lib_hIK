#pragma once
#include "IK_SegClampBase.hpp"
template <typename TSegmentSO3>
class TIK_SegClampDirect : public TIK_SegClampBase<TSegmentSO3>
{
	typedef TIK_SegClampBase<TSegmentSO3> Super;
public:
	TIK_SegClampDirect()
		: MIN_THETA(-(Real)M_PI)
		, MAX_THETA(+(Real)M_PI)
		, MIN_TAU(-(Real)M_PI)
		, MAX_TAU(+(Real)M_PI)
		, MIN_PHI(-(Real)M_PI)
		, MAX_PHI(+(Real)M_PI)
		, m_sin_half_theta_min(-1)
		, m_sin_half_theta_max(+1)
		, m_sin_half_phi_min(-1)
		, m_sin_half_phi_max(-1)
	{
		Super::m_limRange[TSegmentSO3::R_theta][0] = MIN_THETA;
		Super::m_limRange[TSegmentSO3::R_theta][1] = MAX_THETA;

		Super::m_limRange[TSegmentSO3::R_tau][0] = MIN_TAU;
		Super::m_limRange[TSegmentSO3::R_tau][1] = MAX_TAU;

		Super::m_limRange[TSegmentSO3::R_phi][0] = MIN_PHI;
		Super::m_limRange[TSegmentSO3::R_phi][1] = MAX_PHI;
	}

private:
	virtual bool Initialize(CArtiBodyNode* from, CArtiBodyNode* to) override
	{
		if (!Super::Initialize(from, to))
			return false;

		m_sin_half_theta_min = (Real)sin((Real)0.5 * Super::m_limRange[TSegmentSO3::R_theta][0]);
		m_sin_half_theta_max = (Real)sin((Real)0.5 * Super::m_limRange[TSegmentSO3::R_theta][1]);

		m_sin_half_phi_min = (Real)sin((Real)0.5 * Super::m_limRange[TSegmentSO3::R_phi][0]);
		m_sin_half_phi_max = (Real)sin((Real)0.5 * Super::m_limRange[TSegmentSO3::R_phi][1]);
		return true;
	}

	virtual void SetLimit(IK_QSegment::DOFLim dof_l, const Real lims[2]) override
	{
		IKAssert(-1 < dof_l && dof_l < 3);

		const Real validRange[3][2] = {
			{MIN_THETA, MAX_THETA},
			{MIN_TAU, MAX_TAU},
			{MIN_PHI, MAX_PHI},
		};

		if (Super::m_limited[dof_l]
			= (validRange[dof_l][0]<=lims[0] && lims[1] <= validRange[dof_l][1]))
		{
			Super::m_limRange[dof_l][0] = lims[0];
			Super::m_limRange[dof_l][1] = lims[1];
		}
	}

	virtual bool ClampST(bool clamped[3], Eigen::Quaternionr& rotq) override
	{
		memset(clamped, false, sizeof(bool) * 3);
		bool clampping[3] = {
			Super::m_limited[TSegmentSO3::R_theta],
			Super::m_limited[TSegmentSO3::R_tau],
			Super::m_limited[TSegmentSO3::R_phi]
		};
		bool exist_a_lim = ( clampping[0]
							|| clampping[1]
							|| clampping[2] )  ;
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
		Real sin_half_theta_clamp = clampping[TSegmentSO3::R_theta]
									? std::max(m_sin_half_theta_min
											, std::min(m_sin_half_theta_max
														, sin_half_theta))
									: sin_half_theta;


		Real sin_half_phi_clamp =  clampping[TSegmentSO3::R_phi]
									? std::max(m_sin_half_phi_min
											, std::min(m_sin_half_phi_max
														, sin_half_phi))
									: sin_half_phi;

		Real sin_half_tau_clamp = y_t;
		Real cos_half_tau_clamp = w_t;

		bool clampedST[3] = {
						clampping[TSegmentSO3::R_theta] && (sin_half_theta != sin_half_theta_clamp),
						clampping[TSegmentSO3::R_tau] && (TIK_SegClampBase<TSegmentSO3>::ClampT(y_t, w_t, sin_half_tau_clamp, cos_half_tau_clamp)),
						clampping[TSegmentSO3::R_phi] && (sin_half_phi != sin_half_phi_clamp)
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
				t_q.x() = 0;
				t_q.y() = sin_half_tau_clamp;
				t_q.z() = 0;
				clamped[1] = true;
			}
			rotq = s_q * t_q;
			return true;
		}
	}
private:
	const Real MIN_THETA;
	const Real MAX_THETA;
	const Real MIN_TAU;
	const Real MAX_TAU;
	const Real MIN_PHI;
	const Real MAX_PHI;
	// angle in [min, max]

	Real m_sin_half_theta_min, m_sin_half_theta_max;
	Real m_sin_half_phi_min, m_sin_half_phi_max;
};

