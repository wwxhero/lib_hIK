#pragma once
#include "IKChain.hpp"
#include "parallel_thread_helper.hpp"
#include "MotionPipeConf.hpp"

class CIKGroup
{
public:
	CIKGroup(CArtiBodyNode* root);
	CIKGroup(CIKGroup& src);
	~CIKGroup();

private:
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

public:
	bool BeginUpdate(Transform_TR* w2g);
	bool BeginUpdate(const Transform_TR& w2g, const TransformArchive& tm_0);
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

	Real Error() const
	{
		Real err = 0;
		for (auto chain_i : m_kChains)
			err += chain_i->Error();
		return err;
	}

	CIKChain* AddChain(const CONF::CIKChainConf* conf);
private:
	CArtiBodyNode* m_rootBody;
	std::vector<CIKChain*> m_kChains;
};

class CThreadIKGroup : public CThread_W32
{
public:
	CThreadIKGroup();
	~CThreadIKGroup();
	void Initialize_main(const CArtiBodyNode* root_src);
	CIKChain* AddChain_main(const CONF::CIKChainConf* conf);
	void Update_main(const Transform_TR& w2g, const TransformArchive& tm_0);
	bool AcqUpdateRes_main(TransformArchive* tm_k);
	void SetupTargets_main(const std::map<std::wstring, CArtiBodyNode*>& nameSrc2bodyDst, const Eigen::Matrix3r& src2dst_w, const Eigen::Matrix3r& dst2src_w);
	bool Reset()
	{
		bool solved = m_solved;
		m_solved = false;
		return solved;
	}
private:
	virtual void Run_worker();
private:
	volatile bool m_solved;
	CIKGroup* volatile m_group;
	CArtiBodyNode* m_rootParent;
};

class CIKGroupsParallel
{
public:
	CIKGroupsParallel(const CArtiBodyNode* root_src, int concurrency);
	CIKGroupsParallel(CIKGroupsParallel& src);
	~CIKGroupsParallel();
	bool Update_A(const Transform_TR& w2g, const TransformArchive& tm_0);
	bool SolutionFinal(const TransformArchive& tm_0, TransformArchive* tm_star);
	void AddChain(const CONF::CIKChainConf* conf);
	void SetupTargets(const std::map<std::wstring, CArtiBodyNode*>& nameSrc2bodyDst, const Eigen::Matrix3r& src2dst_w, const Eigen::Matrix3r& dst2src_w);
private:
	CThreadPool_W32<CThreadIKGroup> m_pool;
};