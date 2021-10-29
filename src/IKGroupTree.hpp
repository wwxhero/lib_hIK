#pragma once
#include <ostream>
#include "IKChain.hpp"
#include "MotionPipeConf.hpp"
#include "PostureGraph.hpp"

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
		if (multiple_chain)
		{
			const char* warning = "Grouping multiple chain undermines the IK performance!!!";
			LOGIKVarWarning(LogInfoCharPtr, warning);
		}
		chain->SegGRoot(m_rootBody);
	}

	void IKUpdate()
	{
		int n_chains = (int)m_kChains.size();
		if (n_chains < 1)
			return;
		Transform_TR tm_w2g_tr;
		CArtiBodyNode* g_parent = m_rootBody->GetParent();
		if (NULL != g_parent)
		{
			const Transform* tm_w2g = g_parent->GetTransformWorld2Local();
			IKAssert(t_tr == tm_w2g->Type());
			tm_w2g_tr = *static_cast<const Transform_TR*>(tm_w2g);
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

		if (m_pg)
		{
			auto Err = [&]() -> Real
				{
					Real err = 0;
					for (auto chain : m_kChains)
						err += chain->Error();
					return err;
				};
			int theta_min = CFile2PostureGraphClose::LocalMin(*m_pg, Err);
			m_pg->SetActivePosture(theta_min, true);
		}

		bool solved_all = false;
		if (1 == n_chains)
		{
			solved_all = m_kChains[0]->Update();
		}
		else
		{
			for (int i_update = 0; i_update < n_chains && !solved_all; i_update ++)
			{
				for (auto& chain_i : m_kChains)
					chain_i->Update();

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
			m_kChains[i_chain]->EndUpdate();
		}

		CArtiBodyTree::FK_Update<false>(m_rootBody);
	}

	void SetupTargets(const std::map<std::wstring, CArtiBodyNode*>& nameSrc2bodyDst, const Eigen::Matrix3r& src2dst_w, const Eigen::Matrix3r& dst2src_w);
	void LoadPostureGraph(const char* pgDir);
	virtual void Dump(int indent) const override;
	void Dump(int indent, std::ostream& out) const;

	bool Empty() const
	{
		return m_kChains.empty();
	}

	CArtiBodyNode* RootBody() const
	{
		return m_rootBody;
	}
protected:
	CArtiBodyNode* m_rootBody;
	std::vector<CIKChain*> m_kChains;
	int m_nSpecMax;
	CFile2PostureGraphClose* m_pg;
};

class CIKGroupTree : public Tree<CIKGroupNode>
{
public:
	static CIKGroupNode* Generate(const CArtiBodyNode* root, const CONF::CBodyConf& ikChainConf);
	static void SetupTargets(CIKGroupNode* root_ik, const std::map<std::wstring, CArtiBodyNode*>& nameSrc2bodyDst, const Eigen::Matrix3r& src2dst_w, const Eigen::Matrix3r& dst2src_w);
	static void LoadPG(CIKGroupNode* root_ik, const char* dirPath);
};


