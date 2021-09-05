#include "pch.h"
#include "IKGroupTree.hpp"

class CArtiBodyClrNode 				// jTree
	: public TreeNode<CArtiBodyClrNode>
{
public:
	CArtiBodyClrNode(const CArtiBodyNode* body)
		: m_body(body)
		, m_clr(-1)
	{

	}
	void Dump(int n_indents) const
	{
		std::stringstream logInfo;
		for (int i_indent = 0; i_indent < n_indents; i_indent ++)
			logInfo << "\t";
		logInfo << m_body->GetName_c() << ":" << m_clr;
		LOGIK(logInfo.str().c_str());
	}
private:
	int m_clr;
	const CArtiBodyNode* m_body;
};

class CArtiBodyClrTree
	: public Tree<CArtiBodyClrNode>
{
public:
	static CArtiBodyClrNode* Generate(const CArtiBodyNode* root);
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

CIKGroupNode* CIKGroupTree::Generate(const CArtiBodyNode* root, const std::vector<CONF::CIKChainConf>& ikChainConf)
{
	CIKGroupNode* g_root = NULL;
	CArtiBodyClrNode* root_clr = CArtiBodyClrTree::Generate(root);
	if (NULL != root_clr)
	{
#ifdef _DEBUG
		CArtiBodyClrTree::Dump(root_clr);
#endif
	// CArtiBodyClrTree::ColorGid(root_clr, ikChainConf);
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