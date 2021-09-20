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
#include <limits>
#include "IK_QJacobian.h"
#include "ik_logger.h"

#pragma push_macro("min")
#pragma push_macro("max")
#undef min
#undef max

IK_QJacobian::IK_QJacobian()
{
}

IK_QJacobian::~IK_QJacobian()
{
}

void IK_QJacobian::ArmMatrices(int dof, int task_size)
{
  m_dof = dof;
  m_task_size = task_size;

  m_jacobian.resize(task_size, dof);
  m_jacobian.setZero();

  m_nullspace.resize(dof, dof);

  m_d_theta.resize(dof);

  m_beta.resize(task_size);

  m_weight.resize(dof);
  m_weight_sqrt.resize(dof);
  m_weight.setOnes();
  m_weight_sqrt.setOnes();

  if (task_size >= dof) {
    m_transpose = false;

    m_svd_u.resize(task_size, dof);
    m_svd_v.resize(dof, dof);
    m_svd_w.resize(dof);

    m_svd_u_beta.resize(dof);
  }
  else {
    // use the SVD of the transpose jacobian, it works just as well
    // as the original, and often allows using smaller matrices.
    m_transpose = true;

    m_svd_u.resize(task_size, task_size);
    m_svd_v.resize(dof, task_size);
    m_svd_w.resize(task_size);

    m_svd_u_beta.resize(task_size);
  }
  LOGIKVar(LogInfoInt, dof);
  LOGIKVar(LogInfoInt, task_size);
}

void IK_QJacobian::SetBetas(int id, int, const Eigen::Vector3r &v)
{
  m_beta[id + 0] = v.x();
  m_beta[id + 1] = v.y();
  m_beta[id + 2] = v.z();
}

Eigen::Vector3r IK_QJacobian::GetBeta(int id) const
{
  Eigen::Vector3r beta(m_beta[id + 0], m_beta[id + 1], m_beta[id + 2]);
  return beta;
}

int IK_QJacobian::NumBetas() const
{
  int numBetas = (int)m_beta.rows()/3;
  IKAssert(numBetas * 3 == m_beta.rows());
  return numBetas;
}

void IK_QJacobian::SetDerivatives(int id, int dof_id, const Eigen::Vector3r &v)
{
  m_jacobian(id + 0, dof_id) = v.x() * m_weight_sqrt[dof_id];
  m_jacobian(id + 1, dof_id) = v.y() * m_weight_sqrt[dof_id];
  m_jacobian(id + 2, dof_id) = v.z() * m_weight_sqrt[dof_id];
}

void IK_QJacobian::Invert()
{
  LOGIKVar(LogInfoInt, (int)m_jacobian.rows());
  LOGIKVar(LogInfoInt, (int)m_jacobian.cols());

  // std::stringstream m_d_norm_weight_Info;
  // m_d_norm_weight_Info << "\n" << m_d_norm_weight;
  // LOGIKVar(LogInfoCharPtr, m_d_norm_weight_Info.str().c_str());

  std::stringstream m_weight_Info;
  m_weight_Info << "\n" << m_weight;
  LOGIKVar(LogInfoCharPtr, m_weight_Info.str().c_str());

  std::stringstream m_weight_sqrt_Info;
  m_weight_sqrt_Info << "\n" << m_weight_sqrt;
  LOGIKVar(LogInfoCharPtr, m_weight_sqrt_Info.str().c_str());

  if (m_transpose) {
    // SVD will decompose Jt into V*W*Ut with U,V orthogonal and W diagonal,
    // so J = U*W*Vt and Jinv = V*Winv*Ut
    Eigen::JacobiSVD<Eigen::MatrixXr> svd(m_jacobian.transpose(),
                                   Eigen::ComputeThinU | Eigen::ComputeThinV);
    m_svd_u = svd.matrixV();
    m_svd_w = svd.singularValues();
    m_svd_v = svd.matrixU();
  }
  else {
    // SVD will decompose J into U*W*Vt with U,V orthogonal and W diagonal,
    // so Jinv = V*Winv*Ut
    Eigen::JacobiSVD<Eigen::MatrixXr> svd(m_jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    m_svd_u = svd.matrixU();
    m_svd_w = svd.singularValues();
    m_svd_v = svd.matrixV();
  }

  LOGIKVar(LogInfoBool, m_transpose);
  LOGIKVar(LogInfoInt, (int)m_svd_u.rows());
  LOGIKVar(LogInfoInt, (int)m_svd_u.cols());
  LOGIKVar(LogInfoInt, (int)m_svd_w.rows());
  LOGIKVar(LogInfoInt, (int)m_svd_w.cols());
  LOGIKVar(LogInfoInt, (int)m_svd_v.rows());
  LOGIKVar(LogInfoInt, (int)m_svd_v.cols());

}

bool IK_QJacobian::ComputeNullProjection()
{
  const Real epsilon = c_epsilon;

  // compute null space projection based on V
  int i, j, rank = 0;
  for (i = 0; i < m_svd_w.size(); i++)
    if (m_svd_w[i] > epsilon)
      rank++;

  if (rank < m_task_size)
    return false;

  Eigen::MatrixXr basis(m_svd_v.rows(), rank);
  int b = 0;

  for (i = 0; i < m_svd_w.size(); i++)
    if (m_svd_w[i] > epsilon) {
      for (j = 0; j < m_svd_v.rows(); j++)
        basis(j, b) = m_svd_v(j, i);
      b++;
    }

  m_nullspace = basis * basis.transpose();

  for (i = 0; i < m_nullspace.rows(); i++)
    for (j = 0; j < m_nullspace.cols(); j++)
      if (i == j)
        m_nullspace(i, j) = (Real)1.0 - m_nullspace(i, j);
      else
        m_nullspace(i, j) = -m_nullspace(i, j);

  return true;
}

void IK_QJacobian::SubTask(IK_QJacobian &jacobian)
{
  if (!ComputeNullProjection())
    return;

  // restrict lower priority jacobian
  jacobian.Restrict(m_d_theta, m_nullspace);

  // add angle update from lower priority
  jacobian.Invert();

  // note: now damps secondary angles with minimum damping value from
  // SDLS, to avoid shaking when the primary task is near singularities,
  // doesn't work well at all
  int i;
  for (i = 0; i < m_d_theta.size(); i++)
    m_d_theta[i] = m_d_theta[i] + /*m_min_damp * */ jacobian.AngleUpdate(i);
}

void IK_QJacobian::Restrict(Eigen::VectorXr &d_theta, Eigen::MatrixXr &nullspace)
{
  // subtract part already moved by higher task from beta
  m_beta = m_beta - m_jacobian * d_theta;

  // note: should we be using the norm of the unrestricted jacobian for SDLS?

  // project jacobian on to null space of higher priority task
  m_jacobian = m_jacobian * nullspace;
}

void IK_QJacobianSDLS::Invert()
{
  IK_QJacobian::Invert();
  // Compute the dampeds least squeares pseudo inverse of J.
  //
  // Since J is usually not invertible (most of the times it's not even
  // square), the psuedo inverse is used. This gives us a least squares
  // solution.
  //
  // This is fine when the J*Jt is of full rank. When J*Jt is near to
  // singular the least squares inverse tries to minimize |J(dtheta) - dX)|
  // and doesn't try to minimize  dTheta. This results in eratic changes in
  // angle. The damped least squares minimizes |dtheta| to try and reduce this
  // erratic behaviour.
  //
  // The selectively damped least squares (SDLS) is used here instead of the
  // DLS. The SDLS damps individual singular values, instead of using a single
  // damping term.

  const Real max_angle_change = (Real)(M_PI / 4.0);
  const Real epsilon = c_epsilon;
  int i, j;

  m_d_theta.setZero();

  for (i = 0; i < m_dof; i++) {
    m_norm[i] = 0.0;
    for (j = 0; j < m_task_size; j += 3) {
      Real n = 0.0;
      n += m_jacobian(j, i) * m_jacobian(j, i);
      n += m_jacobian(j + 1, i) * m_jacobian(j + 1, i);
      n += m_jacobian(j + 2, i) * m_jacobian(j + 2, i);
      m_norm[i] += sqrt(n);
    }
  }

  for (i = 0; i < m_svd_w.size(); i++) {
    if (m_svd_w[i] <= epsilon)
      continue;

    Real wInv = (Real)1.0 / m_svd_w[i];
    Real alpha = 0.0;
    Real N = 0.0;

    // compute alpha and N
    for (j = 0; j < m_svd_u.rows(); j += 3) {
      alpha += m_svd_u(j, i) * m_beta[j];
      alpha += m_svd_u(j + 1, i) * m_beta[j + 1];
      alpha += m_svd_u(j + 2, i) * m_beta[j + 2];

      // note: for 1 end effector, N will always be 1, since U is
      // orthogonal, .. so could be optimized
      Real tmp;
      tmp = m_svd_u(j, i) * m_svd_u(j, i);
      tmp += m_svd_u(j + 1, i) * m_svd_u(j + 1, i);
      tmp += m_svd_u(j + 2, i) * m_svd_u(j + 2, i);
      N += sqrt(tmp);
    }
    alpha *= wInv;

    // compute M, dTheta and max_dtheta
    Real M = 0.0;
    Real max_dtheta = 0.0, abs_dtheta;

    for (j = 0; j < m_d_theta.size(); j++) {
      Real v = m_svd_v(j, i);
      M += fabs(v) * m_norm[j];

      // compute tmporary dTheta's
      m_d_theta_unclamped[j] = v * alpha;

      // find largest absolute dTheta
      // multiply with weight to prevent unnecessary damping
      abs_dtheta = fabs(m_d_theta_unclamped[j]) * m_weight_sqrt[j];
      if (abs_dtheta > max_dtheta)
        max_dtheta = abs_dtheta;
    }

    M *= wInv;

    // compute damping term and damp the dTheta's
    Real gamma = max_angle_change;
    if (N < M)
      gamma *= N / M;

    Real damp = (gamma < max_dtheta) ? gamma / max_dtheta : (Real)1.0;

    for (j = 0; j < m_d_theta.size(); j++) {
      // slight hack: we do 0.80*, so that if there is some oscillation,
      // the system can still converge (for joint limits). also, it's
      // better to go a little to slow than to far

      Real dofdamp = damp / m_weight[j];
      if (dofdamp > 1.0)
        dofdamp = 1.0;

      m_d_theta[j] += (Real)0.80 * dofdamp * m_d_theta_unclamped[j];
    }
  }

  // weight + prevent from doing angle updates with angles > max_angle_change
  Real max_angle = 0.0, abs_angle;

  for (j = 0; j < m_dof; j++) {
    m_d_theta[j] *= m_weight[j];

    abs_angle = fabs(m_d_theta[j]);

    if (abs_angle > max_angle)
      max_angle = abs_angle;
  }

  if (max_angle > max_angle_change) {
    Real damp = (max_angle_change) / (max_angle_change + max_angle);

    for (j = 0; j < m_dof; j++)
      m_d_theta[j] *= damp;
  }
}

void IK_QJacobianDLS::Invert()
{
  IK_QJacobian::Invert();
  // Compute damped least squares inverse of pseudo inverse
  // Compute damping term lambda

  // Note when lambda is zero this is equivalent to the
  // least squares solution. This is fine when the m_jjt is
  // of full rank. When m_jjt is near to singular the least squares
  // inverse tries to minimize |J(dtheta) - dX)| and doesn't
  // try to minimize  dTheta. This results in eratic changes in angle.
  // Damped least squares minimizes |dtheta| to try and reduce this
  // erratic behaviour.

  // We don't want to use the damped solution everywhere so we
  // only increase lamda from zero as we approach a singularity.

  // find the smallest non-zero W value, anything below epsilon is
  // treated as zero

  const Real epsilon = c_epsilon;
  const Real max_angle_change = (Real)0.1;
  const Real x_length = sqrt(m_beta.dot(m_beta));

  int i, j;
  Real w_min = std::numeric_limits<Real>::max();

  for (i = 0; i < m_svd_w.size(); i++) {
    if (m_svd_w[i] > epsilon && m_svd_w[i] < w_min)
      w_min = m_svd_w[i];
  }

  // compute lambda damping term

  Real d = x_length / max_angle_change;
  Real lambda;

  if (w_min <= d / 2)
    lambda = d / 2;
  else if (w_min < d)
    lambda = sqrt(w_min * (d - w_min));
  else
    lambda = 0.0;

  lambda *= lambda;

  if (lambda > 10)
    lambda = 10;

  LOGIKVar(LogInfoFloat, lambda);
  // immediately multiply with Beta, so we can do matrix*vector products
  // rather than matrix*matrix products

  // compute Ut*Beta
  m_svd_u_beta = m_svd_u.transpose() * m_beta;

  m_d_theta.setZero();

  for (i = 0; i < m_svd_w.size(); i++) {
    if (m_svd_w[i] > epsilon) {
      Real wInv = m_svd_w[i] / (m_svd_w[i] * m_svd_w[i] + lambda);

      // compute V*Winv*Ut*Beta
      m_svd_u_beta[i] *= wInv;
        // m_d_theta = m_d_theta + m_svd_u_beta[i]*m_svd_v(:, i)
      for (j = 0; j < m_d_theta.size(); j++)
        m_d_theta[j] += m_svd_v(j, i) * m_svd_u_beta[i];
    }
  }

  for (j = 0; j < m_d_theta.size(); j++)
    m_d_theta[j] *= m_weight[j];
}

void IK_QJacobian::Lock(int dof_id, Real delta)
{
  int i;

  for (i = 0; i < m_task_size; i++) {
    m_beta[i] -= m_jacobian(i, dof_id) * delta;
    m_jacobian(i, dof_id) = 0.0;
  }

  m_d_theta[dof_id] = 0.0;
}

Real IK_QJacobian::AngleUpdate(int dof_id) const
{
  return m_d_theta[dof_id];
}

Real IK_QJacobian::AngleUpdateNorm() const
{
  int i;
  Real mx = 0.0, dtheta_abs;

  for (i = 0; i < m_d_theta.size(); i++) {
    dtheta_abs = fabs(m_d_theta[i]);
    if (dtheta_abs > mx)
      mx = dtheta_abs;
  }

  return mx;
}

void IK_QJacobian::SetDoFWeight(int dof, Real weight)
{
  m_weight[dof] = weight;
  m_weight_sqrt[dof] = sqrt(weight);
}

IK_QJacobianDLS::IK_QJacobianDLS()
  : IK_QJacobian()
{
}

IK_QJacobianSDLS::IK_QJacobianSDLS()
  : IK_QJacobian()
{

}

void IK_QJacobianSDLS::ArmMatrices(int dof, int task_size)
{
  IK_QJacobian::ArmMatrices(dof, task_size);
  m_d_theta_unclamped.resize(dof);
  m_norm.resize(dof);
  m_norm.setZero();
}

void IK_QJacobianSDLS::Lock(int dof_id, Real delta)
{
  IK_QJacobian::Lock(dof_id, delta);
  m_norm[dof_id] = 0.0;  // unneeded
}


#pragma pop_macro("min")
#pragma pop_macro("max")