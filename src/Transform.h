#pragma once
#include "articulated_body.h"
#include "Math.hpp"

class Transform
{
public:
	virtual Eigen::Matrix3r getLinear() const = 0;
	virtual void setLinear(const Eigen::Matrix3r& tm_l) = 0;
	virtual Eigen::Vector3r getTranslation() const = 0;
	virtual void setTranslation(const Eigen::Vector3r& tt) = 0;
	virtual std::string ToString() const = 0;
	virtual void CopyTo(_TRANSFORM& tm) const = 0;
	virtual void CopyFrom(const _TRANSFORM& tm) = 0;
	virtual Eigen::Matrix3r getRotation_m() const;
	virtual Eigen::Affine3r getAffine() const;
};

class Transform_TRS
	: public Eigen::Affine3r
	, public Transform

{
	typedef Eigen::Affine3r Super;

	inline void Init(const _TRANSFORM& tm)
	{
		Eigen::Vector3r p(tm.tt.x, tm.tt.y, tm.tt.z);
		Eigen::Quaternionr r(tm.r.w, tm.r.x, tm.r.y, tm.r.z);
		Eigen::Vector3r s(tm.s.x, tm.s.y, tm.s.z);
		fromPositionOrientationScale(p, r, s);
	}
public:
	Transform_TRS(const _TRANSFORM& tm)
	{
		Init(tm);
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

	virtual void CopyTo(_TRANSFORM& tm) const;
	virtual void CopyFrom(const _TRANSFORM& tm);
	virtual std::string ToString() const;
	virtual Eigen::Matrix3r getLinear() const;
	virtual void setLinear(const Eigen::Matrix3r& lin);
	virtual Eigen::Vector3r getTranslation() const;
	virtual void setTranslation(const Eigen::Vector3r& tt);
	virtual Eigen::Affine3r getAffine() const;
};

class Transform_T : public Transform
{
	friend class Transform_TR;

private:
	Eigen::Vector3r m_tt;
};

class Transform_R : public Transform
{
	friend class Transform_TR;

	void Init(const _TRANSFORM& tm)
	{
		assert(1 == tm.s.x
			&& 1 == tm.s.y
			&& 1 == tm.s.z);

		m_rotq.w() = tm.r.w;
		m_rotq.x() = tm.r.x;
		m_rotq.y() = tm.r.y;
		m_rotq.z() = tm.r.z;

		assert(0 == tm.tt.x
			&& 0 == tm.tt.y
			&& 0 == tm.tt.z);
	}
public:
	Transform_R(const _TRANSFORM& tm)
	{
		Init(tm);
	}

	Transform_R()
	{
		m_rotq.setIdentity();
	}

	virtual Eigen::Matrix3r getLinear() const;
	virtual void setLinear(const Eigen::Matrix3r& tm_l);
	virtual Eigen::Vector3r getTranslation() const;
	virtual void setTranslation(const Eigen::Vector3r& tt);
	virtual std::string ToString() const;
	virtual void CopyTo(_TRANSFORM& tm) const;
	virtual void CopyFrom(const _TRANSFORM& tm);
	virtual Eigen::Matrix3r getRotation_m() const override;
	inline Transform_R inverse() const
	{
		Transform_R t_inv;
		t_inv.m_rotq = m_rotq.inverse();
		return std::move(t_inv);
	}
	inline const Eigen::Quaternionr& getRotation_q() const
	{
		return m_rotq;
	}
protected:
	Eigen::Quaternionr m_rotq;
};

class Transform_TR : public Transform_R
{
	void Init(const _TRANSFORM& tm)
	{
		assert(1 == tm.s.x
			&& 1 == tm.s.y
			&& 1 == tm.s.z);

		m_rotq.w() = tm.r.w;
		m_rotq.x() = tm.r.x;
		m_rotq.y() = tm.r.y;
		m_rotq.z() = tm.r.z;

		m_tt.x() = tm.tt.x;
		m_tt.y() = tm.tt.y;
		m_tt.z() = tm.tt.z;
	}
public:
	Transform_TR(const _TRANSFORM& tm)
	{
		Init(tm);
	}

	Transform_TR()
	{
		m_rotq.setIdentity();
		m_tt.setZero();
	}

	virtual Eigen::Vector3r getTranslation() const;
	virtual void setTranslation(const Eigen::Vector3r& tt);
	virtual std::string ToString() const;
	virtual void CopyTo(_TRANSFORM& tm) const;
	virtual void CopyFrom(const _TRANSFORM& tm);

	inline Transform_TR inverse() const
	{
		Transform_TR t_inv;
		t_inv.m_rotq = m_rotq.inverse();
		t_inv.m_tt = Eigen::Vector3r::Zero() - t_inv.m_rotq*m_tt;
		return std::move(t_inv);
	}


	inline void Update(const Transform_TR& tm0, const Transform_R& delta)
	{
		m_rotq = tm0.m_rotq * delta.m_rotq;
		m_tt = tm0.m_tt;
	}

	inline void Update(const Transform_TR& tm0, const Transform_T& delta)
	{
		m_rotq = tm0.m_rotq;
		m_tt = tm0.m_rotq*delta.m_tt + tm0.m_tt;
	}

	inline void Update(const Transform_TR& tm0, const Transform_TR& delta)
	{
		m_rotq = tm0.m_rotq * delta.m_rotq;
		m_tt = tm0.m_rotq*delta.m_tt + tm0.m_tt;
	}
private:
	Eigen::Vector3r m_tt;
};
