#pragma once
#include "IKChain.hpp"
class CIKGroup
{
public:
	CIKGroup(CArtiBodyNode* root);
	CIKGroup(CIKGroup& src);
	~CIKGroup();

	int Join(CIKChain* chain)
	{
		auto it_chain = m_kChains.begin();
		for (
			; it_chain != m_kChains.end()
				&& (*it_chain) < chain
			; it_chain ++);
		m_kChains.insert(it_chain, chain);

		// insertion sort with predicate: len(chain_i) <= len(chain_i+1)
		int n_steps_i = chain->NIters();

		bool multiple_chain = (m_kChains.size() > 1);
		if (multiple_chain)
		{
			const char* warning = "Grouping multiple chain undermines the IK performance!!!";
			LOGIKVarWarning(LogInfoCharPtr, warning);
		}
		chain->SetGRoot(m_rootBody);
		return n_steps_i;
	}

	bool BeginUpdate();
	bool Update();
	void EndUpdate();
	void IKReset();

	void SetupTargets(const std::map<std::wstring, CArtiBodyNode*>& nameSrc2bodyDst, const Eigen::Matrix3r& src2dst_w, const Eigen::Matrix3r& dst2src_w);
	void Dump(int indent) const;
	void Dump(int indent, std::ostream& out) const;

	bool Empty() const
	{
		return m_kChains.empty();
	}

	CArtiBodyNode* RootBody() const
	{
		return m_rootBody;
	}
private:
	CArtiBodyNode* m_rootBody;
	std::vector<CIKChain*> m_kChains;
};