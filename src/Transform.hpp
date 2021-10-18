#pragma once
#include <vector>
#include "articulated_body.h"
#include "Math.hpp"
#include "ik_logger.h"


inline bool NoScale(const _TRANSFORM& tm)
{
	auto err_x = tm.s.x - (Real)1;
	auto err_y = tm.s.y - (Real)1;
	auto err_z = tm.s.z - (Real)1;
	return (-c_10epsilon < err_x && err_x < c_10epsilon)
		&& (-c_10epsilon < err_y && err_y < c_10epsilon)
		&& (-c_10epsilon < err_z && err_z < c_10epsilon);

}

inline bool NoRot(const _TRANSFORM& tm)
{
	auto err_w = tm.r.w - (Real)1;
	return (-c_10epsilon < err_w && err_w < c_10epsilon);
}

inline bool NoTT(const _TRANSFORM& tm)
{
	return (-c_10epsilon < tm.tt.x && tm.tt.x < c_10epsilon)
		&& (-c_10epsilon < tm.tt.y && tm.tt.y < c_10epsilon)
		&& (-c_10epsilon < tm.tt.z && tm.tt.z < c_10epsilon);
}

inline bool Is_ID(const _TRANSFORM& tm)
{
	return NoScale(tm) && NoRot(tm) && NoTT(tm);
}

inline void SetID(_TRANSFORM& tm)
{
	tm.s.x = (Real)1; tm.s.y = (Real)1; tm.s.z = (Real)1;
	tm.r.x = (Real)0; tm.r.y = (Real)0; tm.r.z = (Real)0; tm.r.w = (Real)1;
	tm.tt.x = (Real)0; tm.tt.y = (Real)0; tm.tt.z = (Real)0;
}

inline bool Equal(const _TRANSFORM& tm_1, const _TRANSFORM& tm_2)
{
	Real err_s_x = tm_1.s.x - tm_2.s.x;
	Real err_s_y = tm_1.s.y - tm_2.s.y;
	Real err_s_z = tm_1.s.z - tm_2.s.z;

	bool s_eq = (-c_100epsilon < err_s_x && err_s_x < c_100epsilon)
			 && (-c_100epsilon < err_s_y && err_s_y < c_100epsilon)
			 && (-c_100epsilon < err_s_z && err_s_z < c_100epsilon);

	if (!s_eq)
		return false;

	Real err_tt_x = tm_1.tt.x - tm_2.tt.x;
	Real err_tt_y = tm_1.tt.y - tm_2.tt.y;
	Real err_tt_z = tm_1.tt.z - tm_2.tt.z;

	bool tt_eq = (-c_tt_epsilon < err_tt_x && err_tt_x < c_tt_epsilon)
			  && (-c_tt_epsilon < err_tt_y && err_tt_y < c_tt_epsilon)
			  && (-c_tt_epsilon < err_tt_z && err_tt_z < c_tt_epsilon);

	if (!tt_eq)
		return false;

	Real err_r =  tm_1.r.w * tm_2.r.w
				+ tm_1.r.x * tm_2.r.x
				+ tm_1.r.y * tm_2.r.y
				+ tm_1.r.z * tm_2.r.z;

	Real err_r_0 = err_r - (Real)1;
	Real err_r_1 = err_r + (Real)1;

	bool r_eq = (-c_rotq_epsilon < err_r_0 && err_r_0 < c_rotq_epsilon)
			 || (-c_rotq_epsilon < err_r_1 && err_r_1 < c_rotq_epsilon);

	return r_eq;
}

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
	static Eigen::Quaternionr getRotation_q(const Transform*);
	TM_TYPE Type() const
	{
		return c_type;
	}
protected:
	TM_TYPE c_type;
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
		c_type = t_trs;
	}

	Transform_TRS(const Transform_TRS& tm)
		: Super(tm)
	{
		c_type = t_trs;
	}

	Transform_TRS(const Eigen::Affine3r& tm)
		: Super(tm)
	{
		c_type = t_trs;
	}

	Transform_TRS()
	{
		c_type = t_trs;
		setIdentity();
	}

	explicit Transform_TRS(const Real m_affine[3][4])
	{
		c_type = t_trs;
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
	inline void setRotation(const Eigen::Quaternionr& rotq)
	{
		linear() = rotq.matrix();
	}
	inline Eigen::Quaternionr getRotation_q() const
	{
		Eigen::Quaternionr q(getRotation_m());
		return q;
	}
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
		IKAssert(NoScale(tm)
				&& NoTT(tm));

		m_rotq.w() = tm.r.w;
		m_rotq.x() = tm.r.x;
		m_rotq.y() = tm.r.y;
		m_rotq.z() = tm.r.z;
	}
public:
	Transform_R(const _TRANSFORM& tm)
	{
		c_type = t_r;
		Init(tm);
	}

	Transform_R()
	{
		c_type = t_r;
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
		t_inv.m_rotq = m_rotq.conjugate();
		return std::move(t_inv);
	}
	inline const Eigen::Quaternionr& getRotation_q() const
	{
		return m_rotq;
	}
	inline void setRotation(const Eigen::Quaternionr& rotq)
	{
		m_rotq = rotq;
	}
protected:
	Eigen::Quaternionr m_rotq;
};

class Transform_TR : public Transform_R
{
	void Init(const _TRANSFORM& tm)
	{
		IKAssert(NoScale(tm));

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
		c_type = t_tr;
		Init(tm);
	}

	Transform_TR()
	{
		c_type = t_tr;
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
		t_inv.m_rotq = m_rotq.conjugate();
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

	inline Eigen::Vector3r Apply_p(const Eigen::Vector3r& p) const
	{
		return m_rotq * p + m_tt;
	}

	inline Eigen::Vector3r Apply_v(const Eigen::Vector3r& v) const
	{
		return m_rotq * v;
	}

	inline Plane Apply(const Plane& plane) const
	{
		Eigen::Vector3r p_0 = Apply_p(plane.p);
		Eigen::Vector3r n = Apply_v(plane.n);
		return Plane(n, p_0);
	}

	bool Valid() const
	{
		auto abs_rotq = m_rotq.squaredNorm();
		Real err = abs_rotq - (Real)1;
		return -c_rotq_epsilon_sqrnorm < err && err < +c_rotq_epsilon_sqrnorm;
	}

	Transform_TR operator=(const Transform_TR& other)
	{
		m_tt = other.m_tt;
		m_rotq = other.m_rotq;
		return *this;
	}

private:
	Eigen::Vector3r m_tt;
};


class TransformArchive
{
public:
	TransformArchive()
	{
	}
	void Resize(int sz)
	{
		m_data.resize(sz);
	}
	_TRANSFORM& operator[](int i)
	{
		IKAssert(i < (int)m_data.size());
		return m_data[i];
	}
	static Real Error_q(const TransformArchive &tm, const TransformArchive &tm_prime); //in range [0, 1]
private:
	std::vector<_TRANSFORM> m_data;
};