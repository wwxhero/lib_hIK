#include "pch.h"
#include "IK_QSegment.hpp"

BEGIN_ENUM_STR(IK_QSegment, Type)
	ENUM_ITEM(R_xyz)
END_ENUM_STR(IK_QSegment, Type)

IK_QSegment::IK_QSegment(Type a_type, int n_dof)
	: m_scale((Real)1.0)
	, m_bodies{NULL, NULL}
	, m_joints{NULL, NULL}
	, m_max_extension((Real)0)
	, m_DoF_id(-1)
	, c_type(a_type)
	, c_num_DoF(n_dof)
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
	return m_max_extension > c_epsilon;
}

IK_QIxyzSegment::IK_QIxyzSegment()
	: IK_QSegment(R_xyz, 3)
	, m_weight{(Real)1, (Real)1, (Real)1}
	, m_axis {Eigen::Vector3r::UnitX(), Eigen::Vector3r::UnitY(), Eigen::Vector3r::UnitZ()}
	, m_locked {false, false, false}
	, m_theta(Eigen::Vector3r::Zero())
{
}

Real IK_QIxyzSegment::Weight(int dof_l) const
{
	IKAssert(-1 < dof_l && dof_l < 3);
	return m_weight[dof_l];
}

void IK_QIxyzSegment::SetWeight(int dof_l, Real w)
{
	IKAssert(-1 < dof_l && dof_l < 3);
	m_weight[dof_l] = w;
}

Eigen::Vector3r IK_QIxyzSegment::Axis(int dof_l) const
{
	IKAssert(-1 < dof_l && dof_l < 3);
	return m_axis[dof_l];
}

bool IK_QIxyzSegment::Locked(int dof_l) const
{
	IKAssert(-1 < dof_l && dof_l < 3);
	return m_locked[dof_l];
}

void IK_QIxyzSegment::UnLock()
{
	memset(m_locked, false, sizeof(m_locked));
}

void IK_QIxyzSegment::Lock(int dof_l, IK_QJacobian &jacobian, Eigen::Vector3r &delta)
{
	LOGIK("Lock");
	m_locked[dof_l] = true;
	jacobian.Lock(m_DoF_id + dof_l, delta[dof_l]);
}

void IK_QIxyzSegment::FK_Update()
{
	IK_QSegment::FK_Update();
	const Transform* tm_l2w = m_bodies[0]->GetTransformLocal2World();
	Eigen::Matrix3r linear = tm_l2w->getLinear();
	for (int dof_l = 0; dof_l < c_num_DoF; dof_l ++)
		m_axis[dof_l] = linear.col(dof_l);
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

		m_theta += delta;

		// interpret dq as: x->y'->z'' == z->y->x
		Eigen::AngleAxisr rz(m_theta[2], Eigen::Vector3r::UnitZ());
		Eigen::AngleAxisr ry(m_theta[1], Eigen::Vector3r::UnitY());
		Eigen::AngleAxisr rx(m_theta[0], Eigen::Vector3r::UnitX());
		m_joints[0]->SetRotation(rx * ry * rz);
	}

	return false;
}

