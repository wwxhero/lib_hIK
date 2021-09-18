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

IK_QTask::IK_QTask(Type type, int size, bool primary, const IK_QSegment *segment)
    : c_type(type)
    , m_size(size)
    , m_primary(primary)
    , m_segment(segment)
    , m_weight(1.0)
{
}

// IK_QPositionTask

IK_QPositionTask::IK_QPositionTask(bool primary, const IK_QSegment *segment, const Eigen::Vector3r &goal)
    : IK_QTask(Position, 3, primary, segment)
    , m_goal(goal)
{
  // computing clamping length
  int num;
  const IK_QSegment *seg;

  m_clamp_length = 0.0;
  num = 0;

  for (seg = m_segment; seg; seg = seg->Parent()) {
    m_clamp_length += seg->MaxExtension();
    num++;
  }

  m_clamp_length /= 2 * num;
}

void IK_QPositionTask::ComputeJacobian(IK_QJacobian &jacobian)
{
  // compute beta
  const Eigen::Vector3r &pos = m_segment->GlobalEnd();

  Eigen::Vector3r d_pos = m_goal - pos;
  double length = d_pos.norm();

  if (length > m_clamp_length)
    d_pos = (m_clamp_length / length) * d_pos;

  jacobian.SetBetas(m_id, m_size, m_weight * d_pos);

  // compute derivatives
  int i;
  const IK_QSegment *seg;

  for (seg = m_segment; seg; seg = seg->Parent()) {
    Eigen::Vector3r p = seg->GlobalStart() - pos;

    for (i = 0; i < seg->NumberOfDoF(); i++) {
      Eigen::Vector3r axis = seg->Axis(i) * m_weight;
      bool translational = seg->Translational();
      IKAssert(!translational);  // currently we don't have tranlational segement supported
      if (translational)
        jacobian.SetDerivatives(m_id, seg->DoFId() + i, axis);
      else {
        Eigen::Vector3r pa = p.cross(axis);
        jacobian.SetDerivatives(m_id, seg->DoFId() + i, pa);
      }
    }
  }
}

// IK_QOrientationTask

IK_QOrientationTask::IK_QOrientationTask(bool primary,
                                         const IK_QSegment *segment,
                                         const Eigen::Matrix3r &goal)
    : IK_QTask(Orientation, 3, primary, segment)
	, m_goal(goal)
{
}

void IK_QOrientationTask::ComputeJacobian(IK_QJacobian &jacobian)
{
  // compute betas
  const Eigen::Matrix3r &rot = m_segment->GlobalTransform().linear();
  //d_rotm^-1 = m_goal * rot ^-1
  // => d_rotm = rot * m_goal^-1
  // => rot^-1 * d_rotm = m_goal^-1
  // => d_rotm^-1 * rot = m_goal
  Eigen::Matrix3r d_rotm = (m_goal * rot.transpose()).transpose();

  Eigen::Vector3r d_rot;
  d_rot = -0.5 * Eigen::Vector3r(d_rotm(2, 1) - d_rotm(1, 2),
                          d_rotm(0, 2) - d_rotm(2, 0),
                          d_rotm(1, 0) - d_rotm(0, 1));

  jacobian.SetBetas(m_id, m_size, m_weight * d_rot);

  // compute derivatives
  int i;
  const IK_QSegment *seg;

  for (seg = m_segment; seg; seg = seg->Parent())
    for (i = 0; i < seg->NumberOfDoF(); i++) {
      bool translational = seg->Translational();
      IKAssert(!translational);  // currently we don't have tranlational segement supported
      if (translational)
        jacobian.SetDerivatives(m_id, seg->DoFId() + i, Eigen::Vector3r(0, 0, 0));
      else {
        Eigen::Vector3r axis = seg->Axis(i) * m_weight;
        jacobian.SetDerivatives(m_id, seg->DoFId() + i, axis);
      }
    }
}



