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

#pragma once

/**
 * @author Laurence Bourn
 * @date 28/6/2001
 */

#include <vector>
#include <list>

#include "Math.hpp"
#include "IK_QJacobian.h"
#include "IK_QSegment.h"
#include "IK_QTask.h"

class IK_QJacobianSolver {
 public:
  IK_QJacobianSolver();
  ~IK_QJacobianSolver()
  {
  }

  // call setup once before solving, if it fails don't solve
  bool Setup(IK_QSegment *root, std::list<IK_QTask *> &tasks);

  // returns true if converged, false if max number of iterations was used
  bool Solve(IK_QSegment *root,
             std::list<IK_QTask *> tasks,
             const double tolerance,
             const int max_iterations);

 protected:
  void AddSegmentList(IK_QSegment *seg);
  bool UpdateAngles(double &norm);

  double ComputeScale();
  void Scale(double scale, std::list<IK_QTask *> &tasks);

 protected:
  IK_QJacobianSDLS m_jacobian;
  IK_QJacobianSDLS m_jacobian_sub;

  bool m_secondary_enabled;

  std::vector<IK_QSegment *> m_segments;


};


class IK_QJacobianSolverPoleAngle : protected IK_QJacobianSolver
{
public:
  IK_QJacobianSolverPoleAngle();

    // setup pole vector constraint
  void SetPoleVectorConstraint(
      IK_QSegment *tip, Vector3d &goal, Vector3d &polegoal, float poleangle, bool getangle);
  float GetPoleAngle()
  {
    return m_poleangle;
  }
  void ConstrainPoleVector(IK_QSegment *root, std::list<IK_QTask *> &tasks);

  void Scale(double scale, std::list<IK_QTask *> &tasks);

    // call setup once before solving, if it fails don't solve
  bool Setup(IK_QSegment *root, std::list<IK_QTask *> &tasks)
  {
    return IK_QJacobianSolver::Setup(root, tasks);
  }

    // returns true if converged, false if max number of iterations was used
  bool Solve(IK_QSegment *root,
             std::list<IK_QTask *> tasks,
             const double tolerance,
             const int max_iterations);
protected:
  Affine3d m_rootmatrix;

  bool m_poleconstraint;
  bool m_getpoleangle;
  Vector3d m_goal;
  Vector3d m_polegoal;
  float m_poleangle;
  IK_QSegment *m_poletip;
};
