#include "IKChain.hpp"
#include "IK_QJacobian.h"
#include "IK_QSegment.hpp"
#include "IK_QTask.h"

template<typename IK_QJacobianX>
class IKChainInverseJK : public CIKChain
{
public:
	IKChainInverseJK(CIKChain::Algor algor, Real weight_p, Real weight_r, int n_iter, int n_predecessor)
		: CIKChain(algor, n_iter, n_predecessor)
		, m_taskP(true, m_segments, m_eefSrc)
		, m_taskR(true, m_segments, m_eefSrc)
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
		for (auto seg : m_segments)
			delete seg;
	}

	virtual bool Init(const CArtiBodyNode* eef, int len, const std::vector<CONF::CJointConf>& joint_confs) override
	{
		if (!CIKChain::Init(eef, len, joint_confs))
			return false;

		for (auto seg : m_segments)
			delete seg;
		m_segments.clear();

		std::map<std::string, const CONF::CJointConf*> name2confJoint;
		for (auto& confJoint : joint_confs)
			name2confJoint[confJoint.name] = &confJoint;

		int n_nodes = (int)m_nodes.size();

		std::vector<CArtiBodyNode*> seg_from(n_nodes);
		std::vector<CArtiBodyNode*> seg_to(n_nodes);
		int i_node = 0;
		seg_from[i_node] = m_nodes[i_node].body;
		int i_node_m = 0;
		for (i_node = 1; i_node < n_nodes; i_node ++, i_node_m ++)
		{
			auto& node_i = m_nodes[i_node];
			seg_to[i_node_m] = node_i.body;
			seg_from[i_node] = node_i.body;
		}
		seg_to[i_node_m] = const_cast<CArtiBodyNode*>(eef);

		m_segments.resize(n_nodes);
		int n_segs = 0;
		for (i_node = 0; i_node < n_nodes; i_node ++)
		{
			auto it_conf_j = name2confJoint.find(seg_from[i_node]->GetName_c());
			const auto type_default = IK_QSegment::R_xyz;
			const Real dex_default[3] = { (Real)1, (Real)1, (Real)1 };

			IK_QSegment::Type type;
			const Real(*p_dex)[3] = NULL;
			if (name2confJoint.end() != it_conf_j)
			{
				type = it_conf_j->second->type;
				p_dex = &it_conf_j->second->dexterity;
			}
			else
			{
				type = IK_QSegment::R_xyz;
				p_dex = &dex_default;
			}
			IK_QSegment* seg = NULL;
			switch (type)
			{
				case IK_QSegment::R_xyz:
					seg = new IK_QIxyzSegment(*p_dex);
					break;
				default:
					break;
			}
			IKAssert(NULL != seg);
			if (seg->Initialize(seg_from[i_node], seg_to[i_node]))
				m_segments[n_segs ++] = seg;
		}
		m_segments.resize(n_segs);

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
		int i;

		for (seg = m_segments.begin(); seg != m_segments.end(); seg++)
		{
			for (i = 0; i < (*seg)->NumberOfDoF(); i++)
			{
				m_jacobian.SetDoFWeight((*seg)->DoFId() + i, (*seg)->Weight(i));
				LOGIKVar(LogInfoCharPtr, (*seg)->GetName_c());
				LOGIKVar(LogInfoReal, (*seg)->Weight(i));
			}
		}

		return true;
	}

	virtual void Dump(std::stringstream& info) const override
	{
		info << from_Algor(c_algor) << " : ";
		CIKChain::Dump(info);
	}

	virtual bool BeginUpdate(const Transform_TR& w2g) override
	{
		if (!CIKChain::BeginUpdate(w2g))
			return false;
		_TRANSFORM goal;
		m_eefSrc->GetGoal(goal);
		IKAssert(NoScale(goal));

		Eigen::Quaternionr R(goal.r.w, goal.r.x, goal.r.y, goal.r.z);
		Eigen::Vector3r T(goal.tt.x, goal.tt.y, goal.tt.z);

		m_taskP.SetGoal(T);
		m_taskR.SetGoal(R);

		m_scaleNormlize = (Real)1.0; // ComputeScale();
		// Real dt = analyze_time();
		LOGIKVar(LogInfoReal, m_scaleNormlize);
		// Scale(m_scaleNormlize, m_tasksReg);
		return true;
	}

	// virtual void UpdateNext(int step) override;
	// this is a quick IK update solution
	virtual bool UpdateAll()
	{
		// iterate
		bool solved = false;
		bool updating = true;
		int i_iter = 0;
		for (
			; i_iter < m_nIters
				&& !solved
				&& updating
			; i_iter++)
		{
			// root->UpdateTransform(Eigen::Affine3d::Identity());
			std::vector<IK_QTask *>::iterator task;

			// compute jacobian
			for (task = m_tasksReg.begin(); task != m_tasksReg.end(); task++)
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
					const char* err = "IK Exception\n";
					LOGIKVarErr(LogInfoCharPtr, err);
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
			CArtiBodyTree::FK_Update<true>(m_rootG);

			updating = (norm > 1e-3 || i_ter < 10);
			solved = true;
			for (task = m_tasksReg.begin()
				; task != m_tasksReg.end() && solved
				; task++)
				solved = (*task)->Completed();
		}

		auto it_eef_seg = m_segments.end();
		it_eef_seg --;
		LOGIKVar(LogInfoCharPtr, (*it_eef_seg)->GetName_c(1));
		LOGIKVar(LogInfoBool, solved);
		LOGIKVar(LogInfoInt, i_iter);
		return solved;
	}

	virtual void EndUpdate(const Transform_TR& g2w) override
	{
		// Scale(1.0f / m_scaleNormlize, m_tasksReg);
		m_taskP.Complete();
		m_taskR.Complete();
		// analyze_add_run(max_iterations, analyze_time()-dt);
		LOGIKVar(LogInfoInt, m_jacobian.rows());
		LOGIKVar(LogInfoInt, m_jacobian.cols());
	}

protected:
	// true: exists a segment that is locked.
	// false: no segment is locked.
	bool UpdateAngles(Real &norm)
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

	Real ComputeScale()
	{
		std::vector<IK_QSegment *>::iterator seg;
		Real length = (Real)0.0f;

		for (seg = m_segments.begin(); seg != m_segments.end(); seg++)
			length += (*seg)->MaxExtension();

		if (length == 0.0)
			return 1.0;
		else
			return (Real)1.0 / length;
	}

	void Scale(Real scale, std::vector<IK_QTask *> &tasks)
	{
		std::vector<IK_QTask *>::iterator task;
		std::vector<IK_QSegment *>::iterator seg;

		for (task = tasks.begin(); task != tasks.end(); task++)
			(*task)->Scale(scale);

		for (seg = m_segments.begin(); seg != m_segments.end(); seg++)
			(*seg)->Scale(scale);

	}

private:
	IK_QJacobianX m_jacobian;
	IK_QJacobianX m_jacobian_sub;

	IK_QPositionTask m_taskP;
	IK_QOrientationTask m_taskR;
	std::vector<IK_QTask*> m_tasksReg;
	Real m_scaleNormlize;

	bool m_secondary_enabled;

	std::vector<IK_QSegment*> m_segments; //the corresponds to CIKChain::m_segments

};

class CIKChainInverseJK_DLS : public IKChainInverseJK<IK_QJacobianDLS>
{
public:
	CIKChainInverseJK_DLS(Real weight_p, Real weight_r, int n_iter, int n_predecessors);
	virtual ~CIKChainInverseJK_DLS();
};

class CIKChainInverseJK_SDLS : public IKChainInverseJK<IK_QJacobianSDLS>
{
public:
	CIKChainInverseJK_SDLS(Real weight_p, Real weight_r, int n_iter, int n_predecessors);
	virtual ~CIKChainInverseJK_SDLS();
};
