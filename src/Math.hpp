#pragma once

#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif

#include <algorithm>

const Real c_epsilon = 1e-5f;
const Real c_2epsilon = 2e-5f;
const Real c_5epsilon = 5e-5f;
const Real c_10epsilon = 1e-4f;
const Real c_100epsilon = 1e-3f;
const Real c_rotm_epsilon = (Real)0.005;
const Real c_tt_epsilon = (Real)1.5; 				//in centimeter
const Real c_tt_epsilon_sqr = (Real)2.25;
const Real c_rotq_epsilon = (Real)1.0/(Real)180.0;	//err_dot: [1, 0] -> err_deg [0, 180]
const Real c_rotq_epsilon_sqrnorm = (Real)0.002;

const Real c_err_q_epsilon = (Real)1 - (Real)cos((Real)5/(Real)180 * 0.5 * 3.1416);

#pragma push_macro("new")
#undef new

#include <Eigen/Eigenvalues>
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
typedef AngleAxis<Real> AngleAxisr;
} // end namespace Eigen

#pragma pop_macro("new")

void vec_roll_to_mat3_normalized(const Eigen::Vector3r& nor, const Real roll, Eigen::Matrix3r& rotm);

inline bool UnitVec(const Eigen::Vector3r& v)
{
	Real err = v.squaredNorm() - 1;
	return -c_2epsilon < err
			&& err < c_2epsilon;
}

//increasing function in range(codomain) [0, 1]
inline Real Error_q(const Eigen::Quaternionr& q_this, const Eigen::Quaternionr& q_other)
{
	const Real c_err_min = (Real)0;
	const Real c_err_max = (Real)1;
	Real cos_q_q_prime =  q_this.w() * q_other.w()
						+ q_this.x() * q_other.x()
						+ q_this.y() * q_other.y()
						+ q_this.z() * q_other.z();
	cos_q_q_prime = std::min(c_err_max, std::max(c_err_min, abs(cos_q_q_prime)));
	return 1 - cos_q_q_prime;
}

// err_q_epsilon in range [0 1], can be defined as 1-cos(0.5*alpha)
inline bool Equal(const Eigen::Quaternionr& q_this, const Eigen::Quaternionr& q_other, const Real err_q_epsilon = c_err_q_epsilon)
{
	return Error_q(q_this, q_other) < err_q_epsilon;
}

inline bool FuzzyZero(Real x)
{
	return fabs(x) < c_epsilon;
}

template<typename T>
T wrap_pi(T rad)
{
  const T pi_2 = (T)2 * (T)M_PI;
  T rad_wrap = 0;
  auto k = floor(rad / pi_2);
  auto r_i_2pi = rad - k * pi_2;
  assert(r_i_2pi >= 0 && r_i_2pi < pi_2);
  if (r_i_2pi > M_PI)
	rad_wrap = r_i_2pi - pi_2;
  else
	rad_wrap = r_i_2pi;
  return rad_wrap;
}

inline Real rad2deg(Real rad)
{
	const Real r2d = (Real)180 / (Real)M_PI;
	return r2d * wrap_pi(rad);
}

inline Real deg2rad(Real deg)
{
	const Real d2r = (Real)M_PI / (Real)180;
	return wrap_pi(d2r*deg);
}

struct Plane
{
	Eigen::Vector3r n;
	Eigen::Vector3r p;
	Plane(const Eigen::Vector3r& a_n, const Eigen::Vector3r& a_p)
		: n(a_n)
		, p(a_p)
	{
	}
	Plane()
		: n(Eigen::Vector3r::Zero())
		, p(Eigen::Vector3r::Zero())
	{
	}
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