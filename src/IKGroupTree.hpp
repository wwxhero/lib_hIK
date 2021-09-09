#pragma once
#include "ArtiBody.h"
#include "MotionPipeConf.hpp"

class CIKChain
{
public:
	CIKChain();
	bool Init(const CArtiBodyNode* eef, int len);
	void SetupTarget(const std::map<std::wstring, CArtiBodyNode*>& nameSrc2bodyDst
					, const Eigen::Matrix3r& src2dst_w
					, const Eigen::Matrix3r& dst2src_w);

	void Dump(std::stringstream& info) const;
	virtual void BeginUpdate();

	virtual void UpdateNext(int step)
	{
	}

	// this is a quick IK update solution
	virtual void UpdateAll()
	{

	}

	virtual void EndUpdate()
	{

	}

	int NSteps() const
	{
		return m_nSteps;
	}

	int NBodies() const
	{
		return (int)m_bodies.size();
	}
private:
	std::vector<CArtiBodyNode*> m_bodies;
	CArtiBodyNode* m_eefSrc;
	CArtiBodyNode* m_targetDst;
	int m_nSteps;
	Eigen::Matrix3r m_src2dstW_Offset;
	Eigen::Matrix3r m_dst2srcW;
};

class CIKGroupNode : public TreeNode<CIKGroupNode>
{
public:
	CIKGroupNode();
	CIKGroupNode(const CIKGroupNode& src);
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
		int n_steps_i = chain->NSteps();
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


