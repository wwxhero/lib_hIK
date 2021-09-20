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
 * Original author: Laurence
 */

/** \file
 * \ingroup iksolver
 */

#pragma once

#include "Math.hpp"
#include "IK_QJacobian.h"
#include "IK_QSegment.hpp"

class IK_QTask {
 public:
  enum Type { Position, Orientation };
  // segment: is the one prior to end effector
  IK_QTask(Type type, int size, bool primary, const std::vector<IK_QSegment*> &segment);
  virtual ~IK_QTask()
  {
  }

  void SetId(int id)
  {
    m_id = id;
  }

  int Size() const
  {
    return m_size;
  }

  bool Primary() const
  {
    return m_primary;
  }

  Real Weight() const
  {
    return m_weight * m_weight;
  }

  void SetWeight(Real weight)
  {
    m_weight = sqrt(weight);
  }
  // Update Jacobian
  virtual void ComputeJacobian(IK_QJacobian &jacobian) = 0;


  virtual void Scale(Real)
  {
  }

public:
  const Type c_type;
protected:
  int m_id;
  int m_size;
  bool m_primary;
  const std::vector<IK_QSegment*>& m_segments;
  Real m_weight;
};

class IK_QPositionTask : public IK_QTask {
 public:
  IK_QPositionTask(bool primary, const std::vector<IK_QSegment*> &segment);

  void ComputeJacobian(IK_QJacobian &jacobian);

  virtual void Scale(Real scale) override
  {
    m_goal *= scale;
    m_clamp_length *= scale;
  }

  void SetGoal(const Eigen::Vector3r& goal)
  {
    m_goal = goal;
  }

 private:
  Eigen::Vector3r m_goal;
  Real m_clamp_length;
};

class IK_QOrientationTask : public IK_QTask {
 public:
  IK_QOrientationTask(bool primary, const std::vector<IK_QSegment*>& segment);

  void ComputeJacobian(IK_QJacobian &jacobian);

  void SetGoal(const Eigen::Quaternionr& goal)
  {
    m_goal = goal;
  }

 private:
  Eigen::Matrix3r m_goal;
};


