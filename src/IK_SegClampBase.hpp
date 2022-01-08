#pragma once

template<typename TSegmentSO3>
class TIK_SegClampBase : public TSegmentSO3
{
protected:
	TIK_SegClampBase()
		: m_limRange {
						  {std::numeric_limits<Real>::lowest(), std::numeric_limits<Real>::max()}
						, {std::numeric_limits<Real>::lowest(), std::numeric_limits<Real>::max()}
						, {std::numeric_limits<Real>::lowest(), std::numeric_limits<Real>::max()}
					}
		, m_limited {false, false, false}
	{
	}

	virtual bool Initialize(CArtiBodyNode* from, CArtiBodyNode* to) override
	{
		if (!TSegmentSO3::Initialize(from, to))
			return false;
		m_sin_half_tau_min = (Real)sin((Real)0.5 * m_limRange[TSegmentSO3::R_tau][0]);
		m_cos_half_tau_min = (Real)cos((Real)0.5 * m_limRange[TSegmentSO3::R_tau][0]);
		m_sin_half_tau_max = (Real)sin((Real)0.5 * m_limRange[TSegmentSO3::R_tau][1]);
		m_cos_half_tau_max = (Real)cos((Real)0.5 * m_limRange[TSegmentSO3::R_tau][1]);
		return true;
	}

	bool ClampT(const Real &half_s, const Real& half_c
				, Real& half_s_clamp, Real& half_c_clamp)
	{
		IKAssert(m_limited[TSegmentSO3::R_tau]);
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

protected:
	Real m_sin_half_tau_min, m_sin_half_tau_max;
	Real m_cos_half_tau_min, m_cos_half_tau_max;

	bool m_limited[3];
	Real m_limRange[3][2];
};