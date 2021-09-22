#pragma once
#include "IKChain.hpp"
#include "MotionPipeConf.hpp"

class CIKGroupNode : public TreeNode<CIKGroupNode>
{
public:
	CIKGroupNode(const CArtiBodyNode* root);
	explicit CIKGroupNode(CIKGroupNode& src);
	~CIKGroupNode();

	void Joint(CIKChain* chain)
	{
		int len = chain->NBodies();
		auto it_chain = m_kChains.begin();
		for (
			; it_chain != m_kChains.end()
				&& (*it_chain)->NBodies() < len
			; it_chain ++);
		m_kChains.insert(it_chain, chain);

		// insertion sort with predicate: len(chain_i) <= len(chain_i+1)
		int n_steps_i = chain->NIters();
		if (m_nSpecMax < n_steps_i)
			m_nSpecMax = n_steps_i;

		m_updating.push_back(false);
	}

	void IKUpdate()
	{
		int n_chains = (int)m_kChains.size();
		IKAssert(n_chains == m_updating.size());
		if (n_chains < 1)
			return;
		const Transform* tm_w2l = m_rootBody->GetTransformWorld2Local();
		IKAssert(t_tr == tm_w2l->Type());
		const Transform_TR* tm_w2l_tr = static_cast<const Transform_TR*>(tm_w2l);
		bool exist_an_update = false;
		for (int i_chain = 0; i_chain < n_chains; i_chain ++)
		{
			bool updating_i = m_kChains[i_chain]->BeginUpdate(*tm_w2l_tr);
			exist_an_update = (exist_an_update || updating_i);
			m_updating[i_chain] = updating_i;
		}

		if (!exist_an_update)
			return;

		if (1 == n_chains)
		{
			if (m_updating[0])
				m_kChains[0]->UpdateAll();
		}
		else
		{
			for (int i_step = 0; i_step < m_nSpecMax; i_step ++)
			{
				for (int i_chain = 0; i_chain < n_chains; i_chain ++)
				{
					if (m_updating[i_chain])
						m_kChains[i_chain]->UpdateNext(i_step);
				}
			}
		}
		for (int i_chain = 0; i_chain < n_chains; i_chain ++)
		{
			if (m_updating[i_chain])
				m_kChains[i_chain]->EndUpdate();
		}
	}

	void SetupTargets(const std::map<std::wstring, CArtiBodyNode*>& nameSrc2bodyDst, const Eigen::Matrix3r& src2dst_w, const Eigen::Matrix3r& dst2src_w);
	virtual void Dump(int indent) const override;
protected:
	const CArtiBodyNode* m_rootBody;
	std::vector<CIKChain*> m_kChains;
	std::vector<bool> m_updating;
	int m_nSpecMax;
};

class CIKGroupTree : public Tree<CIKGroupNode>
{
public:
	static CIKGroupNode* Generate(const CArtiBodyNode* root, const CONF::CBodyConf& ikChainConf);
	static void SetupTargets(CIKGroupNode* root_ik, const std::map<std::wstring, CArtiBodyNode*>& nameSrc2bodyDst, const Eigen::Matrix3r& src2dst_w, const Eigen::Matrix3r& dst2src_w);
};


