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
		chain->SetGRoot(m_rootBody);
	}

	void IKUpdate();

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


