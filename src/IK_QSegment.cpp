#include "pch.h"
#include "IK_QSegment.hpp"

#define STIFFNESS_EPS ((Real)0.1)

BEGIN_ENUM_STR(IK_QSegment, Type)
	ENUM_ITEM(R_xyz)
	ENUM_ITEM(R_Spherical)
END_ENUM_STR(IK_QSegment, Type)

BEGIN_ENUM_STR(IK_QSegment, DOFLim)
	ENUM_ITEM(R_theta)
	ENUM_ITEM(R_tau)
	ENUM_ITEM(R_phi)
END_ENUM_STR(IK_QSegment, DOFLim)

Real IK_QSegment::MIN_THETA = 0;
Real IK_QSegment::MAX_THETA = (Real)(2*M_PI);

Real IK_QSegment::MIN_PHI = 0;
Real IK_QSegment::MAX_PHI = (Real)M_PI;

Real IK_QSegment::MIN_TAU = -(Real)M_PI;
Real IK_QSegment::MAX_TAU = +(Real)M_PI;


IK_QSegment::IK_QSegment(Type a_type, int n_dofs)
	: m_scale((Real)1.0)
	, m_bodies{NULL, NULL}
	, m_joints{NULL, NULL}
	, m_max_extension((Real)0)
	, m_DoF_id(-1)
	, m_stiffness(STIFFNESS_EPS)
	, c_type(a_type)
	, c_num_DoFs(n_dofs)
	, c_idxFrom(0)
	, c_idxTo(1)
{

}

IK_QSegment::~IK_QSegment()
{

}

bool IK_QSegment::Initialize(CArtiBodyNode* from, CArtiBodyNode* to)
{
	m_bodies[c_idxFrom] = from;
	m_bodies[c_idxTo] = to;
	m_joints[c_idxFrom] = from->GetJoint();
	m_joints[c_idxTo] = to->GetJoint();
	Eigen::Vector3r start_w = GlobalStart();
	Eigen::Vector3r end_w = GlobalEnd();
	m_max_extension = (end_w - start_w).norm();
	return m_max_extension > c_tt_epsilon;
}

IK_QSegmentSO3::IK_QSegmentSO3()
	: IK_QSegment(R_xyz, 3)
	, m_weight{1, 1, 1}
	, m_locked {false, false, false}
	, m_limited {false, false, false}
	, m_limRange {0}
{
	const Real b = 1 - STIFFNESS_EPS;
	const Real k = 2 * STIFFNESS_EPS - 1;
	const Real one_third = (Real)1/(Real)3;
	Real dex = one_third * (m_weight[0] + m_weight[1] + m_weight[2]); // sigma(w) / 3
	m_stiffness = dex * k + b;
}

int IK_QSegmentSO3::Weight(Real w[6]) const
{
	memcpy(w, m_weight, c_num_DoFs * sizeof(Real));
	return c_num_DoFs;
}

void IK_QSegmentSO3::SetWeight(int dof_l, Real w)
{
	IKAssert(-1 < dof_l && dof_l < 3);
	m_weight[dof_l] = w;
}

void IK_QSegmentSO3::SetLimit(DOFLim dof_l, const Real lims[2])
{
	IKAssert(-1 < dof_l && dof_l < 3);
	m_limRange[dof_l][0] = lims[0];
	m_limRange[dof_l][1] = lims[1];

	const Real validRange[3][2] = {
		{IK_QSegment::MIN_THETA, IK_QSegment::MAX_THETA},
		{IK_QSegment::MIN_TAU, IK_QSegment::MAX_TAU},
		{IK_QSegment::MIN_PHI, IK_QSegment::MAX_PHI},
	};

	m_limited[dof_l] = (validRange[dof_l][0]<=lims[0] && lims[1] <= validRange[dof_l][1]);
}

int IK_QSegmentSO3::Axis(Eigen::Vector3r axis[6]) const
{
	const Transform* tm_l2w = m_bodies[0]->GetTransformLocal2World();
	Eigen::Matrix3r linear = tm_l2w->getLinear();
	for (int i_dof = 0; i_dof < c_num_DoFs; i_dof ++)
		axis[i_dof] = linear.col(i_dof);
	return c_num_DoFs;
}

int IK_QSegmentSO3::Locked(bool lock[6]) const
{
	memcpy(lock, m_locked, c_num_DoFs * sizeof(bool));
	return c_num_DoFs;
}

void IK_QSegmentSO3::UnLock()
{
	memset(m_locked, false, sizeof(m_locked));
}

void IK_QSegmentSO3::Lock(int dof_l, IK_QJacobian &jacobian, Eigen::Vector3r &delta)
{
	LOGIK("Lock");
	m_locked[dof_l] = true;
	jacobian.Lock(m_DoF_id + dof_l, delta[dof_l]);
}

bool IK_QSegmentSO3::ClampST(bool clamped[3], Eigen::Quaternionr& rotq)
{
	memset(clamped, false, sizeof(bool)*3);
	bool exist_a_lim = (m_limited[R_theta]
					|| m_limited[R_tau]
					|| m_limited[R_phi]);
	if (!exist_a_lim)
		return false;

	// const Eigen::Quaternionr& rotq0 = rotq;
	Real w = rotq.w();
	Real x = rotq.x();
	Real y = rotq.y();
	Real z = rotq.z();
	Real w_s = sqrt(w*w + y*y), x_s, z_s;	//swing: [w_s, x_s, 0, z_s]
	Real w_t, y_t; 								//twist: [w_t, 0, y_t, 0]
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
	Real theta = atan2(w_s, x_s);
	Real phi = acos(z_s);

	Real v[3] = {theta, tau, phi};

	Real v_clamp[3] = {0};
	bool clampedST[3] = {false};
	for (int i_dof = 0; i_dof < c_num_DoFs; i_dof ++)
	{
		v_clamp[i_dof] = std::max(m_limRange[i_dof][0], std::min(m_limRange[i_dof][1], v[i_dof]));
		clampedST[i_dof] = (v_clamp[i_dof] != v[i_dof]);
	}

	bool clamped_on_swing = (clampedST[R_theta] || clampedST[R_phi]);
	bool clamped_on_twist = (clampedST[R_tau]);
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
			s_q.w() = sin(v_clamp[R_phi])*sin(v_clamp[R_theta]);
			s_q.x() = sin(v_clamp[R_phi])*cos(v_clamp[R_theta]);
			s_q.z() = cos(v_clamp[R_phi]);
			clamped[0] = true;
			clamped[2] = true;
		}
		if (clamped_on_twist)
		{
			Real tau_half = clamped[R_tau]*(Real)0.5;
			t_q.w() = cos(tau_half);
			t_q.y() = sin(tau_half);
			clamped[1] = true;
		}
		rotq = s_q * t_q;
		return true;
	}
}

IK_QIxyzSegment::IK_QIxyzSegment()
	: IK_QSegmentSO3()
{

}



bool IK_QIxyzSegment::UpdateAngle(const IK_QJacobian &jacobian, Eigen::Vector3r &delta, bool *clamp)
{
	if (m_locked[0] && m_locked[1] && m_locked[2])
		return false;

	delta.x() = jacobian.AngleUpdate(m_DoF_id);
	delta.y() = jacobian.AngleUpdate(m_DoF_id + 1);
	delta.z() = jacobian.AngleUpdate(m_DoF_id + 2);

	// Directly update the rotation matrix, with Rodrigues' rotation formula,
	// to avoid singularities and allow smooth integration.
	bool zero_d_theta = ((m_locked[0] || FuzzyZero(delta.x()))
					  && (m_locked[1] || FuzzyZero(delta.y()))
					  && (m_locked[2] || FuzzyZero(delta.z())));
	if (!zero_d_theta)
	{
		Eigen::Quaternionr theta = m_joints[0]->GetRotation();

		// interpret dq as: x->y'->z'' == z->y->x
		Eigen::AngleAxisr rz(delta[2], Eigen::Vector3r::UnitZ());
		Eigen::AngleAxisr ry(delta[1], Eigen::Vector3r::UnitY());
		Eigen::AngleAxisr rx(delta[0], Eigen::Vector3r::UnitX());
		Eigen::Quaternionr theta_prime = theta*rx*ry*rz;
		bool clamped = ClampST(clamp, theta_prime);
		LOGIKVarErr(LogInfoBool, clamped);
		delta = 2 * Eigen::Vector3r(
					theta_prime.x() - theta.x(),
					theta_prime.y() - theta.y(),
					theta_prime.z() - theta.z()
				);
		m_joints[0]->SetRotation(theta_prime);
		return clamped;
	}
	else
		return false;

}

IK_QSphericalSegment::IK_QSphericalSegment()
	: IK_QSegmentSO3()
{

}

bool IK_QSphericalSegment::UpdateAngle(const IK_QJacobian &jacobian, Eigen::Vector3r &delta, bool *clamp)
{
	if (m_locked[0] && m_locked[1] && m_locked[2])
		return false;

	delta.x() = jacobian.AngleUpdate(m_DoF_id);
	delta.y() = jacobian.AngleUpdate(m_DoF_id + 1);
	delta.z() = jacobian.AngleUpdate(m_DoF_id + 2);

	// Directly update the rotation matrix, with Rodrigues' rotation formula,
	// to avoid singularities and allow smooth integration.
	Real delta_theta = delta.norm();
	if (!FuzzyZero(delta_theta))
	{
		Eigen::Vector3r delta_u = delta * (1.0/delta_theta);
		Real delta_theta_half = (Real)0.5 * delta_theta;
		Real cos_theta_half = cos(delta_theta_half);
		Real sin_theta_half = sin(delta_theta_half);
		Eigen::Quaternionr delta_q(cos_theta_half
								, sin_theta_half * delta_u.x()
								, sin_theta_half * delta_u.y()
								, sin_theta_half * delta_u.z());
		Eigen::Quaternionr theta = m_joints[0]->GetRotation();
		Eigen::Quaternionr theta_prime = theta * delta_q;
		bool clamped = ClampST(clamp, theta_prime);
		LOGIKVarErr(LogInfoBool, clamped);
		delta = 2 * Eigen::Vector3r(
					theta_prime.x() - theta.x(),
					theta_prime.y() - theta.y(),
					theta_prime.z() - theta.z()
				);
		m_joints[0]->SetRotation(theta_prime);
		return clamped;
	}
	else
		return false;
}

#undef STIFFNESS_EPS