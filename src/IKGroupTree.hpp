#pragma once
#include "ArtiBody.h"
#include "MotionPipeConf.hpp"
class CIKChain
{
public:
	int N() const
	{
		return 0;
	}
};

class CIKGroupNode : public TreeNode<CIKGroupNode>
{
private:
	std::vector<CIKChain*> m_kChains;
};

class CIKGroupTree : public Tree<CIKGroupNode>
{
public:
	static CIKGroupNode* Generate(const CArtiBodyNode* root, const CONF::CBodyConf& ikChainConf);
};


