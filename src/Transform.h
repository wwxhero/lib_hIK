#pragma once
#include "articulated_body.h"

#include <Eigen/Geometry>

namespace Eigen {
#define EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, Size, SizeSuffix)   \
/** \ingroup matrixtypedefs */                                    \
typedef Matrix<Type, Size, Size> Matrix##SizeSuffix##TypeSuffix;  \
/** \ingroup matrixtypedefs */                                    \
typedef Matrix<Type, Size, 1>    Vector##SizeSuffix##TypeSuffix;  \
/** \ingroup matrixtypedefs */                                    \
typedef Matrix<Type, 1, Size>    RowVector##SizeSuffix##TypeSuffix;

#define EIGEN_MAKE_FIXED_TYPEDEFS(Type, TypeSuffix, Size)         \
/** \ingroup matrixtypedefs */                                    \
typedef Matrix<Type, Size, Dynamic> Matrix##Size##X##TypeSuffix;  \
/** \ingroup matrixtypedefs */                                    \
typedef Matrix<Type, Dynamic, Size> Matrix##X##Size##TypeSuffix;

#define EIGEN_MAKE_TYPEDEFS_ALL_SIZES(Type, TypeSuffix) \
EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 2, 2) \
EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 3, 3) \
EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 4, 4) \
EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, Dynamic, X) \
EIGEN_MAKE_FIXED_TYPEDEFS(Type, TypeSuffix, 2) \
EIGEN_MAKE_FIXED_TYPEDEFS(Type, TypeSuffix, 3) \
EIGEN_MAKE_FIXED_TYPEDEFS(Type, TypeSuffix, 4)

EIGEN_MAKE_TYPEDEFS_ALL_SIZES(Real, r)
EIGEN_MAKE_TYPEDEFS_ALL_SIZES(std::complex<Real>, c)


#undef EIGEN_MAKE_TYPEDEFS_ALL_SIZES
#undef EIGEN_MAKE_TYPEDEFS
#undef EIGEN_MAKE_FIXED_TYPEDEFS

typedef Quaternion<Real> Quaternionr;
typedef Transform<Real,3,Affine> Affine3r;
} // end namespace Eigen

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

	void SetTT(Real x, Real y, Real z)
	{
		m_t.translation() = Eigen::Vector3r(x, y, z);
	}

	bool HasTT() const
	{
		Eigen::Vector3r tt = m_t.translation();
		return tt.norm() > c_epsilon;
	}
private:
	Eigen::Affine3r m_t;
};

