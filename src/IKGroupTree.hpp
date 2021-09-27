#pragma once
#include "IKChain.hpp"
#include "MotionPipeConf.hpp"

class CIKGroupNode : public TreeNode<CIKGroupNode>
{
public:
	CIKGroupNode(CArtiBodyNode* root);
	explicit CIKGroupNode(CIKGroupNode& src);
	~CIKGroupNode();

	void Join(CIKChain* chain)
	{
		auto it_chain = m_kChains.begin();
		for (
			; it_chain != m_kChains.end()
				&& (*it_chain) < chain
			; it_chain ++);
		m_kChains.insert(it_chain, chain);

		// insertion sort with predicate: len(chain_i) <= len(chain_i+1)
		int n_steps_i = chain->NIters();
		if (m_nSpecMax < n_steps_i)
			m_nSpecMax = n_steps_i;

		bool multiple_chain = (m_kChains.size() > 1);
		LOGIKVarWarning(LogInfoBool, multiple_chain);
		chain->SegGRoot(m_rootBody);
	}

	void IKUpdate()
	{
		int n_chains = (int)m_kChains.size();
		if (n_chains < 1)
			return;
		Transform_TR tm_w2g_tr;
		Transform_TR tm_g2w_tr;
		CArtiBodyNode* g_parent = m_rootBody->GetParent();
		if (NULL != g_parent)
		{
			const Transform* tm_w2g = g_parent->GetTransformWorld2Local();
			IKAssert(t_tr == tm_w2g->Type());
			tm_w2g_tr = *static_cast<const Transform_TR*>(tm_w2g);
			const Transform* tm_g2w = g_parent->GetTransformLocal2World();
			IKAssert(t_tr == tm_g2w->Type());
			tm_g2w_tr = *static_cast<const Transform_TR*>(tm_g2w);
		}

		bool exist_an_update = false;
		for (int i_chain = 0; i_chain < n_chains; i_chain ++)
		{
			bool updating_i = m_kChains[i_chain]->BeginUpdate(tm_w2g_tr);
			exist_an_update = (exist_an_update || updating_i);
		}

		if (!exist_an_update)
			return;
		if (NULL != g_parent) //for root of the three FK_Update<G_SPACE=true> has no effect but waist computational resource
			CArtiBodyTree::FK_Update<true>(m_rootBody);

		bool solved_all = false;
		if (1 == n_chains)
		{
			solved_all = m_kChains[0]->UpdateAll();
		}
		else
		{
			for (int i_update = 0; i_update < n_chains && !solved_all; i_update ++)
			{
				for (auto& chain_i : m_kChains)
					chain_i->UpdateAll();

				solved_all = true;
				for (auto chain_i = m_kChains.begin()
					; solved_all && chain_i != m_kChains.end()
					; chain_i ++)
					solved_all = (*chain_i)->UpdateCompleted();
			}
		}
		LOGIKVar(LogInfoBool, solved_all);

		for (int i_chain = 0; i_chain < n_chains; i_chain ++)
		{
			m_kChains[i_chain]->EndUpdate(tm_g2w_tr);
		}

		CArtiBodyTree::FK_Update<false>(m_rootBody);
	}

	void SetupTargets(const std::map<std::wstring, CArtiBodyNode*>& nameSrc2bodyDst, const Eigen::Matrix3r& src2dst_w, const Eigen::Matrix3r& dst2src_w);
	virtual void Dump(int indent) const override;
protected:
	CArtiBodyNode* m_rootBody;
	std::vector<CIKChain*> m_kChains;
	int m_nSpecMax;
};

class CIKGroupTree : public Tree<CIKGroupNode>
{
public:
	static CIKGroupNode* Generate(const CArtiBodyNode* root, const CONF::CBodyConf& ikChainConf);
	static void SetupTargets(CIKGroupNode* root_ik, const std::map<std::wstring, CArtiBodyNode*>& nameSrc2bodyDst, const Eigen::Matrix3r& src2dst_w, const Eigen::Matrix3r& dst2src_w);
};


