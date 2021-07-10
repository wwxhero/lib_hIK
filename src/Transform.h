#pragma once
#include "articulated_body.h"
#include "Math.hpp"

class Transform
{
public:
	virtual Eigen::Matrix3r Linear() const = 0;
	virtual Eigen::Vector3r Translation() const = 0;
	virtual std::string ToString() const = 0;
};

class Transform_TRS
	: protected Eigen::Affine3r
	, public Transform

{
	friend class CArtiBodyNode;
	typedef Eigen::Affine3r Super;
public:
	Transform_TRS(const _TRANSFORM& tm)
	{
		Eigen::Vector3r p(tm.tt.x, tm.tt.y, tm.tt.z);
		Eigen::Quaternionr r(tm.r.w, tm.r.x, tm.r.y, tm.r.z);
		Eigen::Vector3r s(tm.s.x, tm.s.y, tm.s.z);
		fromPositionOrientationScale(p, r, s);
	}

	Transform_TRS(const Transform_TRS& tm)
		: Super(tm)
	{
	}

	Transform_TRS(const Eigen::Affine3r& tm)
		: Super(tm)
	{
	}

	Transform_TRS()
	{
		setIdentity();
	}

	explicit Transform_TRS(const Real m_affine[3][4])
	{
		Real* dat = data();
		for (int i_r = 0; i_r < 3; i_r ++)
		{
			for (int i_c = 0; i_c < 4; i_c ++)
			{
				int i_offset = i_c * 4 + i_r;
				dat[i_offset] = m_affine[i_r][i_c];
			}
		}
		makeAffine();
	}

	void Initialize(const Eigen::Matrix3r& l, const Eigen::Vector3r& tt)
	{
		linear() = l;
		translation() = tt;
	}

	void CopyTo(_TRANSFORM& tm) const
	{
		Eigen::Matrix3r r_m, s_m;
		computeRotationScaling(&r_m, &s_m);
		Eigen::Quaternionr r(r_m);
		Eigen::Vector3r s(s_m.diagonal());
		Eigen::Vector3r t(translation());

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

	Transform_TRS operator* (const Transform_TRS& other) const
	{
		Transform_TRS ret(Super::operator*(other));
		return ret;
	}

	Transform_TRS inverse() const
	{
		Transform_TRS ret(Super::inverse());
		return ret;
	}

	virtual std::string ToString() const
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
		return std::move(strInfo);
	}

	virtual Eigen::Matrix3r Linear() const
	{
		return linear();
	}

	virtual Eigen::Vector3r Translation() const
	{
		return translation();
	}

	virtual bool HasTT() const
	{
		Eigen::Vector3r tt = translation();
		return tt.norm() > c_epsilon;
	}

	static Transform_TRS Scale(Real s);
};

class RotScale : public Transform
{

};
