
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
  virtual void ArmMatrices(int dof, int task_size);
  void SetDoFWeight(int dof, Real weight);

  // Iteratively called
  void SetBetas(int id, int size, const Eigen::Vector3r &v);
  void SetDerivatives(int id, int dof_id, const Eigen::Vector3r &v);
  Eigen::Vector3r GetBeta(int id) const;
  int NumBetas() const;
protected:
  void Invert();
public:

  Real AngleUpdate(int dof_id) const;
  Real AngleUpdateNorm() const;

  // DoF locking for inner clamping loop
  virtual void Lock(int dof_id, Real delta);

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

  int m_dof;
  int m_task_size; // either 3 for position task of a chain, or 6 for position and orientation task
  bool m_transpose; // m_transpose = (m_task_size < m_dof)

  // the jacobian matrix and it's null space projector
  Eigen::MatrixXr m_jacobian;
  Eigen::MatrixXr m_nullspace;

  /// the vector of intermediate betas
  Eigen::VectorXr m_beta;

  /// the vector of computed angle changes
  Eigen::VectorXr m_d_theta;

  /// space required for SVD computation
  Eigen::VectorXr m_svd_w;
  Eigen::MatrixXr m_svd_v;
  Eigen::MatrixXr m_svd_u;

  Eigen::VectorXr m_svd_u_beta;

  // dof weighting
  Eigen::VectorXr m_weight;
  Eigen::VectorXr m_weight_sqrt;
};

class IK_QJacobianDLS : public IK_QJacobian
{
public:
  IK_QJacobianDLS();
  void Invert();
};

class IK_QJacobianSDLS : public IK_QJacobian
{
public:
  IK_QJacobianSDLS();
  virtual void ArmMatrices(int dof, int task_size) override;
  virtual void Lock(int dof_id, Real delta) override;
  void Invert();
private:
  // space required for SDLS
  Eigen::VectorXr m_norm;
  Eigen::VectorXr m_d_theta_unclamped;
};
