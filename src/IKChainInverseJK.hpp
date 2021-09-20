#include "IKChain.hpp"
#include "IK_QJacobian.h"
#include "IK_QSegment.hpp"
#include "IK_QTask.h"

template<typename IK_QJacobianX>
class CIKChainInverseJK : public CIKChain
{
public:
	CIKChainInverseJK(CIKChain::Algor algor, Real weight_p, Real weight_r, int n_iter)
		: CIKChain(algor, n_iter)
	{
		if (weight_p > 0)
		{
			IK_QTask* task = new IK_QPositionTask(true, weight_p);
			m_tasks.push_back(task);
		}

		if (weight_r > 0)
		{
			IK_QTask* task = new IK_QOrientationTask(true, weight_r);
			m_tasks.push_back(task);
		}
	}

	virtual ~CIKChainInverseJK()
	{
		for (auto seg : m_segments)
			delete seg;
		for (auto task : m_tasks)
			delete task;
	}

	virtual bool Init(const CArtiBodyNode* eef, int len, const std::vector<CONF::CJointConf>& joint_confs) override
	{
		if (!CIKChain::Init(eef, len, joint_confs))
			return false;

		for (auto seg : m_segments)
			delete seg;
		m_segments.clear();

		std::map<std::string, CONF::CJointConf&> name2confJoint;
		for (auto confJoint : joint_confs)
			name2confJoint[confJoint.name] = confJoint;

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
			IK_QSegment::Type type = (name2confJoint.end() != it_conf_j)
									? it_conf_j->second.type
									: IK_QSegment::R_xyz;
			IK_QSegment* seg = NULL;
			switch (type)
			{
				case IK_QSegment::R_xyz:
					seg = new IK_QIxyzSegment();
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
		std::vector<IK_QTask*>& tasks = m_tasks;

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
			for (i = 0; i < (*seg)->NumberOfDoF(); i++)
				m_jacobian.SetDoFWeight((*seg)->DoFId() + i, (*seg)->Weight(i));

		return true;
	}

	virtual void Dump(std::stringstream& info) const override
	{
		info << from_Algor(c_algor) << " : ";
		CIKChain::Dump(info);
	}

	// virtual void UpdateNext(int step) override;
	// this is a quick IK update solution
	// virtual void UpdateAll() override;

private:
	IK_QJacobianX m_jacobian;
	IK_QJacobianX m_jacobian_sub;

	std::vector<IK_QTask*> m_tasks;

	bool m_secondary_enabled;

	std::vector<IK_QSegment*> m_segments; //the corresponds to CIKChain::m_segments

};

class CIKChainInverseJK_DLS : public CIKChainInverseJK<IK_QJacobianDLS>
{
public:
	CIKChainInverseJK_DLS(Real weight_p, Real weight_r, int n_iter);
	virtual ~CIKChainInverseJK_DLS();
};

class CIKChainInverseJK_SDLS : public CIKChainInverseJK<IK_QJacobianSDLS>
{
public:
	CIKChainInverseJK_SDLS(Real weight_p, Real weight_r, int n_iter);
	virtual ~CIKChainInverseJK_SDLS();
};
