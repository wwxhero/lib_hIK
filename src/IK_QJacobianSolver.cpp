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
#include "ik_logger.h"
#include "IK_QJacobianSolver.h"

//#include "analyze.h"
IK_QJacobianSolver::IK_QJacobianSolver()
{

}

Real IK_QJacobianSolver::ComputeScale()
{
	return 1.0;
	std::vector<IK_QSegment *>::iterator seg;
	Real length = (Real)0.0f;

	for (seg = m_segments.begin(); seg != m_segments.end(); seg++)
		length += (*seg)->MaxExtension();

	if (length == 0.0)
	return 1.0;
	else
	return (Real)1.0 / length;
}

void IK_QJacobianSolver::Scale(Real scale, std::list<IK_QTask *> &tasks)
{
	std::list<IK_QTask *>::iterator task;
	std::vector<IK_QSegment *>::iterator seg;

	for (task = tasks.begin(); task != tasks.end(); task++)
	(*task)->Scale(scale);

	for (seg = m_segments.begin(); seg != m_segments.end(); seg++)
	(*seg)->Scale(scale);

}

bool IK_QJacobianSolver::Setup(const std::vector<CIKChain::IKNode>& chain
							, const CArtiBodyNode* eef
							, std::list<IK_QTask*> &tasks)
{
	m_segments.clear();

	// int n_nodes = (int)chain.size();
	// m_segments.resize(n_nodes);

	// std::vector<CIKChain::IKNode*> seg_from(n_nodes);
	// std::vector<CIKChain::IKNode*> seg_to(n_nodes);
	// int i_node = 0;
	// seg_from[i_node] = &chain[i_node];
	// int i_node_m = 0;
	// for (i_node = 1; i_node < n_nodes; i_node ++, i_node_m ++)
	// {
	// 	const auto& node_i = chain[i_node];
	// 	seg_to[i_node_m] = node_i;
	// 	seg_from[i_node] = node_i;
	// }
	// seg_to[i_node_m].to = {eef, eef->GetJoint()};

	// m_segments.resize(n_nodes);
	// int n_segs = 0;
	// for (i_node = 0; i_node < n_nodes; i_node ++)
	// {
	// 	if (m_segments[n_segs].Initialize(*seg_from[i_node], *seg_to[i_node]))
	// 		n_segs ++;
	// }
	// m_segments.resize(n_segs);


	// assign each segment a unique id for the jacobian
	std::vector<IK_QSegment*>::iterator seg;
	int num_dof = 0;

	for (seg = m_segments.begin(); seg != m_segments.end(); seg ++)
	{
		(*seg)->SetDoFId(num_dof);
		num_dof += (*seg)->NumberOfDoF();
	}

	if (num_dof == 0)
		return false;

	// compute task id's and assing weights to task
	int primary_size = 0, primary = 0;
	int secondary_size = 0, secondary = 0;
	Real primary_weight = 0.0, secondary_weight = 0.0;
	std::list<IK_QTask *>::iterator task;

	for (task = tasks.begin(); task != tasks.end(); task ++)
	{
		IK_QTask *qtask = *task;

		if (qtask->Primary())
		{
			qtask->SetId(primary_size);
			primary_size += qtask->Size();
			primary_weight += qtask->Weight();
			primary++;
		}
		else
		{
			qtask->SetId(secondary_size);
			secondary_size += qtask->Size();
			secondary_weight += qtask->Weight();
			secondary++;
		}
	}

	if (primary_size == 0 || FuzzyZero(primary_weight))
		return false;

	m_secondary_enabled = (secondary > 0);

	// rescale weights of tasks to sum up to 1
	Real primary_rescale = (Real)1.0 / primary_weight;
	Real secondary_rescale;
	if (FuzzyZero(secondary_weight))
		secondary_rescale = 0.0;
	else
		secondary_rescale = (Real)1.0 / secondary_weight;

	for (task = tasks.begin(); task != tasks.end(); task++)
	{
		IK_QTask *qtask = *task;

		if (qtask->Primary())
			qtask->SetWeight(qtask->Weight() * primary_rescale);
		else
			qtask->SetWeight(qtask->Weight() * secondary_rescale);
	}

	// set matrix sizes
	m_jacobian.ArmMatrices(num_dof, primary_size);
	if (secondary > 0)
		m_jacobian_sub.ArmMatrices(num_dof, secondary_size);

	// set dof weights
	int i;

	for (seg = m_segments.begin(); seg != m_segments.end(); seg++)
		for (i = 0; i < (*seg)->NumberOfDoF(); i++)
			m_jacobian.SetDoFWeight((*seg)->DoFId() + i, (*seg)->Weight(i));

	return true;
}

// true: exists a segment that is locked.
// false: no segment is locked.
bool IK_QJacobianSolver::UpdateAngles(Real &norm)
{
	// assing each segment a unique id for the jacobian
	std::vector<IK_QSegment *>::iterator seg;
	IK_QSegment *qseg, *minseg = NULL;
	Real minabsdelta = 1e10, absdelta;
	Eigen::Vector3r delta, mindelta;
	bool locked = false, clamp[3];
	int i, mindof = 0;

	// here we check if any angle limits were violated. angles whose clamped
	// position is the same as it was before, are locked immediate. of the
	// other violation angles the most violating angle is rememberd
	for (seg = m_segments.begin(); seg != m_segments.end(); seg++)
	{
		qseg = *seg;
		if (qseg->UpdateAngle(m_jacobian, delta, clamp))
		{
			for (i = 0; i < qseg->NumberOfDoF(); i++)
			{
				if (clamp[i] && !qseg->Locked(i))
				{
					absdelta = fabs(delta[i]);
					if (absdelta < c_epsilon)
					{
						qseg->Lock(i, m_jacobian, delta);
						locked = true;
					}
					else if (absdelta < minabsdelta)
					{
						minabsdelta = absdelta;
						mindelta = delta;
						minseg = qseg;
						mindof = i;
					}
				}
			}
		}
	}

	// lock most violating angle
	if (minseg)
	{
		minseg->Lock(mindof, m_jacobian, mindelta);
		locked = true;

		if (minabsdelta > norm)
			norm = minabsdelta;
	}

	if (locked == false)
	{
		// no locking done, last inner iteration, apply the angles
		for (seg = m_segments.begin(); seg != m_segments.end(); seg++)
		{
			(*seg)->UnLock();
			(*seg)->UpdateAngleApply();
		}
	}

	// signal if another inner iteration is needed
	return locked;
}

bool IK_QJacobianSolver::Solve(std::list<IK_QTask *> &tasks
							, const Real tolerance
							, const int max_iterations)
{
	// m_segments[0]->FK_Update();

	float scale = ComputeScale();
	bool solved = false;
	// Real dt = analyze_time();
	LOGIKVar(LogInfoFloat, scale);
	Scale(scale, tasks);

	// iterate
	int iterations = 0;
	for (; iterations < max_iterations; iterations++)
	{
		// root->UpdateTransform(Eigen::Affine3d::Identity());
		std::list<IK_QTask *>::iterator task;

		// compute jacobian
		for (task = tasks.begin(); task != tasks.end(); task++)
		{
			bool primary_tsk = (*task)->Primary();
			// LOGIKVar(LogInfoBool, primary_tsk);
			if (primary_tsk)
				(*task)->ComputeJacobian(m_jacobian);
			else
				(*task)->ComputeJacobian(m_jacobian_sub);
		}

		Real norm = 0.0;

		do
		{
			// invert jacobian
			try
			{
				m_jacobian.Invert();
				if (m_secondary_enabled)
					m_jacobian.SubTask(m_jacobian_sub);
			}
			catch (...)
			{
				fprintf(stderr, "IK Exception\n");
				return false;
			}
			// update angles and check limits
		} while (UpdateAngles(norm));

		// unlock segments again after locking in clamping loop
		std::vector<IK_QSegment*>::iterator seg;
		for (seg = m_segments.begin(); seg != m_segments.end(); seg++)
			(*seg)->UnLock();

		// compute angle update norm
		Real maxnorm = m_jacobian.AngleUpdateNorm();
		if (maxnorm > norm)
			norm = maxnorm;

		// check for convergence
		if (norm < 1e-3 && iterations > 10)
		{
			solved = true;
		}

		m_segments[0]->FK_Update();
	}

	Scale(1.0f / scale, tasks);

	// analyze_add_run(max_iterations, analyze_time()-dt);
	LOGIKVar(LogInfoInt, m_jacobian.rows());
	LOGIKVar(LogInfoInt, m_jacobian.cols());
	LOGIKVar(LogInfoBool, solved);
	LOGIKVar(LogInfoInt, iterations);
	return solved;
}


