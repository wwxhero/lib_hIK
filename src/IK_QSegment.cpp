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

BEGIN_ENUM_STR(IK_QSegment, TypeClamp)
	ENUM_ITEM(C_None)
	ENUM_ITEM(C_Spherical)
	ENUM_ITEM(C_Direct)
END_ENUM_STR(IK_QSegment, TypeClamp)

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
{

}

bool IK_QSegmentSO3::Initialize(CArtiBodyNode* from, CArtiBodyNode* to)
{
	if (!IK_QSegment::Initialize(from, to))
		return false;
	Real sigma_weight = 0;
	for (int i_dof = 0; i_dof < c_num_DoFs; i_dof ++)
		sigma_weight += m_weight[i_dof];
	if (sigma_weight < c_epsilon)
		return false;

	const Real b = 1 - STIFFNESS_EPS;
	const Real k = 2 * STIFFNESS_EPS - 1;
	const Real one_third = (Real)1/(Real)3;
	Real dex = one_third * (m_weight[0] + m_weight[1] + m_weight[2]); // sigma(w) / 3
	m_stiffness = dex * k + b;
	return true;
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
		// LOGIKVarErr(LogInfoBool, clamped);
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