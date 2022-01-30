/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2001-2002 by NaN Holding BV.
 * All rights reserved.
 */

/** \file
 * \ingroup iksolver
 */
#include "pch.h"
#include "IK_QTask.h"
#include "ik_logger.h"


// IK_QTask

IK_QTask::IK_QTask(Type type
				, int size
				, bool primary
				, CArtiBodyNode*& eef)
		: c_type(type)
		, m_size(size)
		, m_primary(primary)
		, m_weight(1.0)
		, m_eef(eef)
{
}

// IK_QPositionTask

IK_QPositionTask::IK_QPositionTask(bool primary
								, CArtiBodyNode*& eef)
		: IK_QTask(Position, 3, primary, eef)
{
}

void IK_QPositionTask::ComputeJacobian(IK_QJacobian &jacobian)
{
	// compute beta
	Eigen::Vector3r pos = m_eef->GetTransformLocal2World()->getTranslation();

	Eigen::Vector3r d_pos = m_goal - pos;
	double length = d_pos.norm();

	if (length > m_clamp_length)
		d_pos = (m_clamp_length / length) * d_pos;

	jacobian.SetBetas(m_id, m_size, m_weight * d_pos);

	// compute derivatives
	Eigen::Vector3r axis[6];
	for (const IK_QSegment* seg : m_segments)
	{
		Eigen::Vector3r p = seg->GlobalStart() - pos;
		int n_dof = seg->Axis(axis);
		for (int i = 0; i < n_dof; i++) {
			Eigen::Vector3r axis_i = axis[i] * m_weight;
			bool translational = seg->Translational();
			IKAssert(!translational);  // currently we don't have tranlational segement supported
			if (translational)
				jacobian.SetDerivatives(m_id, seg->DoFId() + i, axis_i);
			else {
				Eigen::Vector3r pa = p.cross(axis_i);
				jacobian.SetDerivatives(m_id, seg->DoFId() + i, pa);
			}
		}
	}
}

bool IK_QPositionTask::Completed() const
{
	const Transform* l2w = m_eef->GetTransformLocal2World();
	Eigen::Vector3r tt_eef = l2w->getTranslation();
	Real dist_sqr = (tt_eef - m_goal).squaredNorm();

	LOGIKVarNJK(LogInfoCharPtr, m_eef->GetName_c());
	LOGIKVarNJK(LogInfoReal, dist_sqr);

	const Real c_errMaxSqr = 9; // err < 3 cm
	return dist_sqr < c_errMaxSqr;
}

Eigen::Vector3r IK_QPositionTask::Beta() const
{
	const Transform* l2w = m_eef->GetTransformLocal2World();
	Eigen::Vector3r tt_eef = l2w->getTranslation();
	Eigen::Vector3r d_pos = m_goal - tt_eef;
#if defined _DEBUG
	LOGIKVar(LogInfoCharPtr, m_eef->GetName_c());
	Real dist_sqr = d_pos.squaredNorm();
	LOGIKVar(LogInfoReal, dist_sqr);
#endif
	return d_pos;
}

void IK_QPositionTask::SetSegment(const std::vector<IK_QSegment*>& segments)
{
	m_segments = segments;
	// computing clamping length
	int num;

	m_clamp_length = 0.0;
	num = 0;

	for (auto seg : m_segments) {
		m_clamp_length += seg->MaxExtension();
		num++;
	}

	m_clamp_length /= 2 * num;
}


// IK_QOrientationTask

IK_QOrientationTask::IK_QOrientationTask(bool primary
									, CArtiBodyNode*& eef)
		: IK_QTask(Orientation, 3, primary, eef)
{
}

void IK_QOrientationTask::ComputeJacobian(IK_QJacobian &jacobian)
{
	//d_rotm^-1 = m_goal * rot ^-1
	// => d_rotm = rot * m_goal^-1
	// => rot^-1 * d_rotm = m_goal^-1
	// => d_rotm^-1 * rot = m_goal
	Eigen::Quaternionr rot_eef = Transform::getRotation_q(m_eef->GetTransformLocal2World());
	Eigen::Quaternionr rot_eef_inv = rot_eef.conjugate(); // conjugate of a normalized quaternion is the same as invert
	Eigen::Quaternionr d_rotq = m_goalQ * rot_eef_inv;
	Eigen::Vector3r d_rot_rv = 2 * d_rotq.w() * Eigen::Vector3r(d_rotq.x(), d_rotq.y(), d_rotq.z());

	jacobian.SetBetas(m_id, m_size, m_weight * d_rot_rv);
	// compute derivatives
	Eigen::Vector3r axis[6];
	for (auto seg : m_segments)
	{
		bool translational = seg->Translational();
		IKAssert(!translational);  // currently we don't have tranlational segement supported
		int n_dof = seg->Axis(axis);
		for (int i = 0; i < n_dof; i++)
		{
			if (translational)
				jacobian.SetDerivatives(m_id, seg->DoFId() + i, Eigen::Vector3r(0, 0, 0));
			else
			{
				Eigen::Vector3r axis_i = axis[i] * m_weight;
				jacobian.SetDerivatives(m_id, seg->DoFId() + i, axis_i);
			}
		}
	}
}

void IK_QOrientationTask::End()
{
	// IKAssert(Completed());  unconditionaly end task for primary group
	int n_segs = (int)m_segments.size();
	if (n_segs > 0)
	{
		Eigen::Quaternionr goal(m_goalQ);
		m_eef->GetJoint()->SetRotation_w(goal);
	}
}

bool IK_QOrientationTask::Completed() const
{
	Eigen::Quaternionr rot_eef = Transform::getRotation_q(m_eef->GetTransformLocal2World());
	Real err = Error_q(rot_eef, m_goalQ);

	LOGIKVarNJK(LogInfoCharPtr, m_eef->GetName_c());
	LOGIKVarNJK(LogInfoReal, err);

	// const Real c_errMax = (Real)0.0152; // (Real)(1 - cos(deg2rad(20*0.5)));
	// const Real c_errMax = (Real)0.0038; // (Real)(1 - cos(deg2rad(10*0.5)));
	const Real c_errMax = (Real)0.008555; // (Real)(1 - cos(deg2rad(15*0.5)));
	return err < c_errMax;
}



