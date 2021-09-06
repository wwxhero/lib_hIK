#include "pch.h"
#include "IKGroupTree.hpp"

class CArtiBodyClrNode 				// jTree
	: public TreeNode<CArtiBodyClrNode>
{
public:
	CArtiBodyClrNode(const CArtiBodyNode* body)
		: c_body(body)
		, m_clr(-1)
	{

	}
	void Dump(int n_indents) const
	{
		std::stringstream logInfo;
		for (int i_indent = 0; i_indent < n_indents; i_indent ++)
			logInfo << "\t";
		logInfo << c_body->GetName_c() << ":" << m_clr;
		LOGIK(logInfo.str().c_str());
	}

	int GetColor() const
	{
		return m_clr;
	}

	void SetColor(int clr)
	{
		m_clr = clr;
	}

private:
	int m_clr;
public:
	const CArtiBodyNode* c_body;
};

class CIKChainClr
{
public:
	CIKChainClr(CArtiBodyClrNode* eef, int len, int clr, int dist2root)
		: c_eef(eef)
		, c_len(len)
		, m_Color(clr)
		, c_dist2root(dist2root)
	{
	}
	int GetColor() const
	{
		return m_Color;
	}
	void SetColor(int clr)
	{
		m_Color = clr;
	}
	CArtiBodyClrNode* EEF() const
	{
		return c_eef;
	}
public:
	int m_Color;
	CArtiBodyClrNode* c_eef;
	int c_len;
	int c_dist2root;
};

class CArtiBodyClrTree
	: public Tree<CArtiBodyClrNode>
{
public:
	static CArtiBodyClrNode* Generate(const CArtiBodyNode* root);
	static void ColorGid(CArtiBodyClrNode* root_clr, const CONF::CBodyConf& bodyConf);
};

CArtiBodyClrNode* CArtiBodyClrTree::Generate(const CArtiBodyNode* root)
{
	auto GenerateNode = [] (const CArtiBodyNode* src, CArtiBodyClrNode** dst) -> bool
						{
							*dst = new CArtiBodyClrNode(src);
							return true;
						};

	CArtiBodyClrNode* root_clr = NULL;
	bool constructed = Construct(root, &root_clr, GenerateNode);
	LOGIKVar(LogInfoBool, constructed);
	IKAssert(constructed || NULL == root_clr);
	return root_clr;
}

void CArtiBodyClrTree::ColorGid(CArtiBodyClrNode* root_clr, const CONF::CBodyConf& bodyConf)
{
	std::vector<CIKChainClr> chains;
	int dist2root = 0;
	auto OnNode = [&dist2root] (CArtiBodyClrNode* node_this)
				{
					dist2root++;
				};

	auto OffNode = [bodyConf, &chains, &dist2root] (CArtiBodyClrNode* node_this)
				{
					dist2root--;
					int i_eef = bodyConf.Name2IKChainIdx(node_this->c_body->GetName_c());
					if (i_eef > -1)
					{
						CIKChainClr chain(node_this, bodyConf.IK_Chains[i_eef].len, i_eef, dist2root);
						chains.push_back(chain);
					}
				};

	TraverseDFS(root_clr, OnNode, OffNode);

	std::sort(chains.begin()
			, chains.end()
			, [] (const CIKChainClr& a, const CIKChainClr& b) -> bool { return a.c_dist2root > b.c_dist2root; }
			);

	for (const CIKChainClr& chain : chains)
	{
		int clr = chain.GetColor();
		int n = chain.c_len;
		int i_end = n + 1;
		CArtiBodyClrNode* p_i_node = NULL;
		int i = 0;
		for (i = 0, p_i_node = chain.c_eef
			; i < i_end && NULL != p_i_node
			; i ++, p_i_node = p_i_node->GetParent())
		{
			int clr_prime = p_i_node->GetColor();
			if (clr_prime > -1)
				clr = clr_prime;
		}

		for (i = 0, p_i_node = chain.c_eef
			; i < i_end && NULL != p_i_node
			; i ++, p_i_node = p_i_node->GetParent())
		{
			p_i_node->SetColor(clr);
		}
	}

}


class CIKGroupNodeGen 		// G
	: public CIKGroupNode
{
private:
	CArtiBodyClrNode* m_jTree;
};

class CIKGroupTreeGen
	: public CIKGroupTree
{

};

CIKGroupNode* CIKGroupTree::Generate(const CArtiBodyNode* root, const CONF::CBodyConf& ikChainConf)
{
	CIKGroupNode* g_root = NULL;
	CArtiBodyClrNode* root_clr = CArtiBodyClrTree::Generate(root);
	if (NULL != root_clr)
	{
		CArtiBodyClrTree::ColorGid(root_clr, ikChainConf);
#ifdef _DEBUG
		CArtiBodyClrTree::Dump(root_clr);
#endif
	// if (root_clr.Colored())
	// {
	// 	CIKGroupNodeGen* gen_root = CIKGroupTreeGen::Generate(root_clr);
	// 	CIKGroupTreeGen::InitKChain(gen_root);
	// 	g_root = CIKGroupTreeGen::Generate(gen_root);
	// 	CIKGroupTreeGen::Destroy(gen_root);
	// }
		CArtiBodyClrTree::Destroy(root_clr);
	}
	return g_root;
}