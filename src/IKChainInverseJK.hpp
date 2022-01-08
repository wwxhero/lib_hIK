#include "IKChainNumerical.hpp"
#include "IK_QJacobian.h"
#include "IK_QTask.h"

template<typename IK_QJacobianX>
class IKChainInverseJK : public CIKChainNumerical
{
	typedef CIKChainNumerical Super;
public:
	IKChainInverseJK(CIKChain::Algor algor, Real weight_p, Real weight_r, int n_iter)
		: Super(algor, n_iter, weight_p, weight_r)
		, m_taskP(true, m_eefSrc)
		, m_taskR(true, m_eefSrc)
	{
		if (weight_p > 0)
			m_tasksReg.push_back(&m_taskP);
		if (weight_r > 0)
			m_tasksReg.push_back(&m_taskR);
		m_taskP.SetWeight(weight_p);
		m_taskR.SetWeight(weight_r);
	}

	virtual ~IKChainInverseJK()
	{
	}

	virtual bool Init(const CArtiBodyNode* eef, int len, const std::vector<CONF::CJointConf>& joint_confs) override
	{
		if (!Super::Init(eef, len, joint_confs))
			return false;

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
		std::vector<IK_QTask*>::iterator task;
		std::vector<IK_QTask*>& tasks = m_tasksReg;

		for (task = tasks.begin(); task != tasks.end(); task++)
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
		Real weight[6] = {0};
		for (auto seg : m_segments)
		{
			int n_dofs = seg->Weight(weight);
			int i_dof_base = seg->DoFId();
			for (int i_dof = 0; i_dof < n_dofs; i_dof ++)
			{
				m_jacobian.SetDoFWeight(i_dof_base + i_dof, weight[i_dof]);
				LOGIKVar(LogInfoCharPtr, seg->GetName_c());
				LOGIKVar(LogInfoReal, weight[i_dof]);
			}
		}

		m_taskP.SetSegment(m_segments);
		m_taskR.SetSegment(m_segments);

		return true;
	}

	virtual void Dump(std::ostream& info) const override
	{
		info << from_Algor(c_algor) << " : ";
		Super::Dump(info);
	}

	virtual bool BeginUpdate(const Transform_TR& w2g) override
	{
		if (!Super::BeginUpdate(w2g))
			return false;
		_TRANSFORM goal;
		m_eefSrc->GetGoal(goal);
		IKAssert(NoScale(goal));

		Eigen::Quaternionr R(goal.r.w, goal.r.x, goal.r.y, goal.r.z);
		Eigen::Vector3r T(goal.tt.x, goal.tt.y, goal.tt.z);

		m_taskP.SetGoal(T);
		m_taskR.SetGoal(R);

		// Real dt = analyze_time();
		return true;
	}

	// virtual void UpdateNext(int step) override;
	// this is a quick IK update solution
	virtual bool Update_AnyThread()
	{
		// iterate
		auto it_eef_seg = m_segments.end();
		it_eef_seg --;
		LOGIKVarErr(LogInfoCharPtr, (*it_eef_seg)->GetName_c(1));
		Real err = Error();
		LOGIKVarErr(LogInfoReal, err);
		const Real sigma_d_alpha_sqr_min = (Real)0.0000761544202225; // deg2rad(0.5)^2
		bool updating = true;
		int i_iter = 0;
		for (
			; i_iter < m_nIters
				&& updating
			; i_iter++)
		{
			std::vector<IK_QTask *>::iterator task;

			// compute jacobian
			for (task = m_tasksReg.begin(); task != m_tasksReg.end(); task++)
			{
				bool primary_tsk = (*task)->Primary();
				if (primary_tsk)
					(*task)->ComputeJacobian(m_jacobian);
				else
					(*task)->ComputeJacobian(m_jacobian_sub);
			}

			Real sigma_d_alpha_sqr = 0.0;

			// invert jacobian
			try
			{
				m_jacobian.Invert();
				if (m_secondary_enabled)
					m_jacobian.SubTask(m_jacobian_sub);
			}
			catch (...)
			{
				const char* err = "IK Exception\n";
				LOGIKVarErr(LogInfoCharPtr, err);
				return false;
			}
			// update angles and check limits
			UpdateAngles(sigma_d_alpha_sqr);
			LOGIKVarErr(LogInfoReal, sigma_d_alpha_sqr);

			// check for convergence
			err = Error();
			LOGIKVarErr(LogInfoReal, err);

			updating = (sigma_d_alpha_sqr_min < sigma_d_alpha_sqr);
			if (updating)
				CArtiBodyTree::FK_Update<true>(m_rootG);

		}

		for (auto seg : m_segments)
			seg->UnLock();

		bool solved = true;
		for (task = m_tasksReg.begin()
			; task != m_tasksReg.end() && solved
			; task++)
			solved = (*task)->Completed();
		LOGIKVarErr(LogInfoBool, solved);
		LOGIKVarErr(LogInfoInt, i_iter);
		return solved;
	}

	virtual bool UpdateCompleted() const
	{
		bool completed = true;
		for (auto task = m_tasksReg.begin()
			; task != m_tasksReg.end() && completed
			; task ++)
				completed = (*task)->Completed();
		return completed;
	}

	virtual void EndUpdate() override
	{
		m_taskP.Complete();
		m_taskR.Complete();
		// analyze_add_run(max_iterations, analyze_time()-dt);
		LOGIKVar(LogInfoInt, m_jacobian.rows());
		LOGIKVar(LogInfoInt, m_jacobian.cols());
	}

protected:
	// true: lock a segment or segments.
	// false: lock no segment.
	bool UpdateAngles(Real &sigma_d_alpha_sqr)
	{
		// assing each segment a unique id for the jacobian
		std::vector<IK_QSegment *>::iterator seg;
		IK_QSegment *minseg = NULL;
		Real minabsdelta = std::numeric_limits<Real>::max();
		Eigen::Vector3r delta, mindelta;
		bool locked = false, clamp[3];
		int mindof = 0;

		// here we check if any angle limits were violated. angles whose clamped
		// position is the same as it was before, are locked immediate. of the
		// other violation angles the most violating angle is rememberd
		sigma_d_alpha_sqr = 0;
		bool locked_dofs[6] = { false };
		int n_clamp = 0;
		for (auto qseg : m_segments)
		{
			delta.x() = 0; delta.y() = 0; delta.z() = 0;
			if (qseg->UpdateAngle(m_jacobian, delta, clamp))
			{
				int n_dofs = qseg->Locked(locked_dofs);
				for (int i_dof = 0; i_dof < n_dofs; i_dof++)
				{
					if (clamp[i_dof] && !locked_dofs[i_dof])
					{
						Real absdelta = fabs(delta[i_dof]);
						if (absdelta < c_epsilon)
						{
							qseg->Lock(i_dof, m_jacobian, delta);
							locked = true;
						}
						else if (absdelta < minabsdelta)
						{
							minabsdelta = absdelta;
							mindelta = delta;
							minseg = qseg;
							mindof = i_dof;
						}
					}
				}
			}
			sigma_d_alpha_sqr += delta.squaredNorm();
			for (int i_clamp = 0; i_clamp < 3; i_clamp ++)
				if (clamp[i_clamp])
					n_clamp ++;
		}

		LOGIKVarErr(LogInfoInt, n_clamp);

		// lock most violating angle
		if (minseg)
		{
			minseg->Lock(mindof, m_jacobian, mindelta);
			locked = true;
		}

		//if (!locked)
		//{
		//	// no locking done, last inner iteration, apply the angles
		//	for (seg = m_segments.begin(); seg != m_segments.end(); seg++)
		//	{
		//		(*seg)->UnLock();
		//	}
		//}

		// signal if another inner iteration is needed
		return locked;
	}

private:
	IK_QJacobianX m_jacobian;
	IK_QJacobianX m_jacobian_sub;

	IK_QPositionTask m_taskP;
	IK_QOrientationTask m_taskR;
	std::vector<IK_QTask*> m_tasksReg;

	bool m_secondary_enabled;



};

class CIKChainInverseJK_DLS : public IKChainInverseJK<IK_QJacobianDLS>
{
public:
	CIKChainInverseJK_DLS(Real weight_p, Real weight_r, int n_iter);
	virtual ~CIKChainInverseJK_DLS();
};

class CIKChainInverseJK_SDLS : public IKChainInverseJK<IK_QJacobianSDLS>
{
public:
	CIKChainInverseJK_SDLS(Real weight_p, Real weight_r, int n_iter);
	virtual ~CIKChainInverseJK_SDLS();
};
