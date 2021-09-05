#include "pch.h"
#include "IKGroupTree.hpp"

class CArtiBodyClrNode 				// jTree
	: public TreeNode<CArtiBodyClrNode>
{
private:
	int m_clr;
	CArtiBodyNode* m_body;
};

class CArtiBodyClrTree
	: public Tree<CArtiBodyNode>
{

};



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
	// CArtiBodyClrNode* root_clr = CArtiBodyClrTree::Generate(root);
	// CArtiBodyClrTree::ColorGid(root_clr, ikChainConf);
	// if (root_clr.Colored())
	// {
	// 	CIKGroupNodeGen* gen_root = CIKGroupTreeGen::Generate(root_clr);
	// 	CIKGroupTreeGen::InitKChain(gen_root);
	// 	g_root = CIKGroupTreeGen::Generate(gen_root);
	// 	CIKGroupTreeGen::Destroy(gen_root);
	// }
	// CArtiBodyClrTree::Destroy(root_clr);
	return g_root;
}