#pragma once
#include "IKChain.hpp"
#include "MotionPipeConf.hpp"

class CIKGroupNode : public TreeNode<CIKGroupNode>
{
public:
	CIKGroupNode();
	CIKGroupNode(const CIKGroupNode& src);
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
	}
	void IKUpdate()
	{
		int n_chains = (int)m_kChains.size();
		if (n_chains < 1)
			return;

		for (CIKChain* chain : m_kChains)
			chain->BeginUpdate();

		if (1 == n_chains)
			m_kChains[0]->UpdateAll();
		else
		{
			for (int i_step = 0; i_step < m_nSpecMax; i_step ++)
			{
				for (CIKChain* chain : m_kChains)
					chain->UpdateNext(i_step);
			}
		}
		for (CIKChain* chain : m_kChains)
			chain->EndUpdate();
	}
	void SetupTargets(const std::map<std::wstring, CArtiBodyNode*>& nameSrc2bodyDst, const Eigen::Matrix3r& src2dst_w, const Eigen::Matrix3r& dst2src_w);
	virtual void Dump(int indent) const override;
protected:
	std::vector<CIKChain*> m_kChains;
	int m_nSpecMax;
};

class CIKGroupTree : public Tree<CIKGroupNode>
{
public:
	static CIKGroupNode* Generate(const CArtiBodyNode* root, const CONF::CBodyConf& ikChainConf);
	static void SetupTargets(CIKGroupNode* root_ik, const std::map<std::wstring, CArtiBodyNode*>& nameSrc2bodyDst, const Eigen::Matrix3r& src2dst_w, const Eigen::Matrix3r& dst2src_w);
};


