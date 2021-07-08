#pragma once
#include "articulated_body.h"
#include "Math.hpp"


class CTransform
{
public:
	CTransform(const _TRANSFORM& tm)
	{
		Eigen::Vector3r p(tm.tt.x, tm.tt.y, tm.tt.z);
		Eigen::Quaternionr r(tm.r.w, tm.r.x, tm.r.y, tm.r.z);
		Eigen::Vector3r s(tm.s.x, tm.s.y, tm.s.z);
		m_t.fromPositionOrientationScale(p, r, s);
	}

	CTransform(const CTransform& tm)
	{
		m_t = tm.m_t;
	}

	CTransform()
	{
		m_t = Eigen::Affine3r::Identity();
	}

	explicit CTransform(const Real m_affine[3][4])
	{
		Real* data = m_t.data();
		for (int i_r = 0; i_r < 3; i_r ++)
		{
			for (int i_c = 0; i_c < 4; i_c ++)
			{
				int i_offset = i_c * 4 + i_r;
				data[i_offset] = m_affine[i_r][i_c];
			}
		}
		m_t.makeAffine();
	}

	void Initialize(const Eigen::Matrix3r& l, const Eigen::Vector3r& tt)
	{
		m_t.linear() = l;
		m_t.translation() = tt;
	}

	void CopyTo(_TRANSFORM& tm) const
	{
		Eigen::Matrix3r r_m, s_m;
		m_t.computeRotationScaling(&r_m, &s_m);
		Eigen::Quaternionr r(r_m);
		Eigen::Vector3r s(s_m.diagonal());
		Eigen::Vector3r t(m_t.translation());

		tm.s.x = s.x();
		tm.s.y = s.y();
		tm.s.z = s.z();

		tm.r.w = r.w();
		tm.r.x = r.x();
		tm.r.y = r.y();
		tm.r.z = r.z();

		tm.tt.x = t.x();
		tm.tt.y = t.y();
		tm.tt.z = t.z();
	}

	CTransform operator* (const CTransform& other) const
	{
		CTransform ret;
		ret.m_t = m_t * other.m_t;
		return ret;
	}

	CTransform inverse() const
	{
		CTransform ret;
		ret.m_t = m_t.inverse();
		return ret;
	}

	std::string ToString() const
	{
		_TRANSFORM tm;
		CopyTo(tm);
		const unsigned int szBuff = 1024;
		char info[szBuff] = {0};
		sprintf_s(info, szBuff, "s[%.4f\t%.4f\t%.4f], r[%.4f\t%.4f\t%.4f\t%.4f], tt[%.4f\t%.4f\t%.4f]"
					, tm.s.x, tm.s.y, tm.s.z
					, tm.r.w, tm.r.x, tm.r.y, tm.r.z
					, tm.tt.x, tm.tt.y, tm.tt.z);
		std::string strInfo(info);
		return strInfo;
	}

	Eigen::Matrix3r Linear() const
	{
		return m_t.linear();
	}

	void SetTT(Real x, Real y, Real z)
	{
		m_t.translation() = Eigen::Vector3r(x, y, z);
	}

	const Eigen::Vector3r GetTT() const
	{
		return m_t.translation();
	}

	bool HasTT() const
	{
		Eigen::Vector3r tt = m_t.translation();
		return tt.norm() > c_epsilon;
	}

	static CTransform Scale(Real s);
private:
	Eigen::Affine3r m_t;
};

