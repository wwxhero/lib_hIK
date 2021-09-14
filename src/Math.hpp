#pragma once

#pragma push_macro("new")
#undef new

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

#pragma pop_macro("new")

void vec_roll_to_mat3_normalized(const Eigen::Vector3r& nor, const Real roll, Eigen::Matrix3r& rotm);

inline bool UnitVec(const Eigen::Vector3r& v)
{
	Real err = v.squaredNorm() - 1;
	return -c_2epsilon < err
					&& err < c_2epsilon;
}

struct Plane
{
	Eigen::Vector3r n;
	Eigen::Vector3r p;
	Eigen::Vector3r ProjP(const Eigen::Vector3r& a_p) const
	{
		Real t = (p - a_p).transpose() * n;
		return a_p + t*n;
	}
	Eigen::Vector3r ProjV(const Eigen::Vector3r& a_v) const
	{
		Eigen::Vector3r Proj_n_v = (a_v.transpose()*n)*n;
		return a_v - Proj_n_v;
	}

	bool in_P(const Eigen::Vector3r& a_p, Real epsilon = c_epsilon) const
	{
		Real err = (a_p - p).transpose() * n;
		return -epsilon < err && err < epsilon;
	}

	bool in_V(const Eigen::Vector3r& a_v, Real epsilon = c_epsilon) const
	{
		Real err = a_v.transpose() * n;
		return -epsilon < err && err < epsilon;
	}
};