
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

#include "Math.hpp"

class IK_QJacobian {
 public:
  IK_QJacobian();
  ~IK_QJacobian();

  // Call once to initialize
  void ArmMatrices(int dof, int task_size);
  void SetDoFWeight(int dof, Real weight);

  // Iteratively called
  void SetBetas(int id, int size, const Eigen::Vector3r &v);
  void SetDerivatives(int id, int dof_id, const Eigen::Vector3r &v, Real norm_weight);
  Eigen::Vector3r GetBeta(int id) const;
  int NumBetas() const;

  void Invert();

  Real AngleUpdate(int dof_id) const;
  Real AngleUpdateNorm() const;

  // DoF locking for inner clamping loop
  void Lock(int dof_id, Real delta);

  // Secondary task
  bool ComputeNullProjection();

  void Restrict(Eigen::VectorXr &d_theta, Eigen::MatrixXr &nullspace);
  void SubTask(IK_QJacobian &jacobian);

  int rows() const
  {
    return (int)m_jacobian.rows();
  }

  int cols() const
  {
    return (int)m_jacobian.cols();
  }
 private:
  void InvertSDLS();
  void InvertDLS();

  int m_dof;
  int m_task_size; // either 3 for position task of a chain, or 6 for position and orientation task
  bool m_transpose; // m_transpose = (m_task_size < m_dof)

  // the jacobian matrix and it's null space projector
  Eigen::MatrixXr m_jacobian, m_jacobian_tmp;
  Eigen::MatrixXr m_nullspace;

  /// the vector of intermediate betas
  Eigen::VectorXr m_beta;

  /// the vector of computed angle changes
  Eigen::VectorXr m_d_theta;
  Eigen::VectorXr m_d_norm_weight;

  /// space required for SVD computation
  Eigen::VectorXr m_svd_w;
  Eigen::MatrixXr m_svd_v;
  Eigen::MatrixXr m_svd_u;

  Eigen::VectorXr m_svd_u_beta;

  // space required for SDLS

  bool m_sdls;
  Eigen::VectorXr m_norm;
  Eigen::VectorXr m_d_theta_tmp;
  Real m_min_damp;

  // null space task vector
  Eigen::VectorXr m_alpha;

  // dof weighting
  Eigen::VectorXr m_weight;
  Eigen::VectorXr m_weight_sqrt;
};
