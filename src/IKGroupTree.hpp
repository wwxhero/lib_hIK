#pragma once
#include "ArtiBody.h"
#include "MotionPipeConf.hpp"
class CIKChain
{
public:
	CIKChain();
	bool Init(const CArtiBodyNode* eef, int len);
	void Dump(std::stringstream& info) const;
private:
	std::vector<CArtiBodyNode*> m_bodies;
	CArtiBodyNode* m_eef;
};

class CIKGroupNode : public TreeNode<CIKGroupNode>
{
public:
	CIKGroupNode();
	CIKGroupNode(const CIKGroupNode& src);
	void Joint(CIKChain* chain)
	{
		m_kChains.push_back(chain);
	}
	virtual void Dump(int indent) const override;
protected:
	std::vector<CIKChain*> m_kChains;
};

class CIKGroupTree : public Tree<CIKGroupNode>
{
public:
	static CIKGroupNode* Generate(const CArtiBodyNode* root, const CONF::CBodyConf& ikChainConf);
};


