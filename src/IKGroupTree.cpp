#include "pch.h"
#include "IKGroupTree.hpp"

CIKChain::CIKChain()
	: m_eef(NULL)
{
}

bool CIKChain::Init(const CArtiBodyNode* eef, int len)
{
	m_eef = const_cast<CArtiBodyNode*>(eef);
	m_bodies.resize(len);
	CArtiBodyNode* body_p = NULL;
	int i_start = len - 1;
	int i_end = -1;
	int i;
	for ( i = i_start, body_p = m_eef->GetParent()
		; i > i_end && NULL != body_p
		; i --, body_p = body_p->GetParent())
	{
		m_bodies[i] = body_p;
	}

	bool initialized = (i == i_end);
	IKAssert(initialized);
	return initialized;
}

void CIKChain::Dump(std::stringstream& info) const
{
	info << "{";
	for (auto body : m_bodies)
		info <<" "<< body->GetName_c();
	info << " " << m_eef->GetName_c();
	info << "}";
}

CIKGroupNode::CIKGroupNode()
{
}

CIKGroupNode::CIKGroupNode(const CIKGroupNode& src)
{
	m_kChains = src.m_kChains;
}

void CIKGroupNode::Dump(int n_indents) const
{
	std::stringstream logInfo;
	for (int i_indent = 0; i_indent < n_indents; i_indent ++)
		logInfo << "\t";
	logInfo << "{";
		for (auto chain : m_kChains)
		{
			chain->Dump(logInfo);
		}
	logInfo << "}";
	LOGIK(logInfo.str().c_str());
}


#define COLOR_BOTTOM -1

class CArtiBodyClrNode 				// jTree
	: public TreeNode<CArtiBodyClrNode>
{
public:
	CArtiBodyClrNode(const CArtiBodyNode* body)
		: c_body(body)
		, m_clr(COLOR_BOTTOM)
	{

	}
	virtual void Dump(int n_indents) const override
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

	bool Colored() const
	{
		return m_clr > COLOR_BOTTOM;
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

	void Dump() const
	{
		LOGIKVar(LogInfoInt, m_Color);
		LOGIKVar(LogInfoCharPtr, c_eef->c_body->GetName_c());
		LOGIKVar(LogInfoInt, c_len);
		LOGIKVar(LogInfoInt, c_dist2root);
	}

	CIKChain* Generate() const
	{
		CIKChain* chain = new CIKChain();
		if (!chain->Init(c_eef->c_body, c_len))
		{
			delete chain;
			chain = NULL;
		}
		return chain;
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
	static void ColorGid(CArtiBodyClrNode* root_clr, const CONF::CBodyConf& bodyConf, std::vector<CIKChainClr>& chains);
	static bool ValidClr(int clr);
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

bool CArtiBodyClrTree::ValidClr(int clr)
{
	return clr > COLOR_BOTTOM;
}

void CArtiBodyClrTree::ColorGid(CArtiBodyClrNode* root_clr
								, const CONF::CBodyConf& bodyConf
								, std::vector<CIKChainClr>& chains)
{
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

	for (CIKChainClr& chain : chains)
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
			if (CArtiBodyClrTree::ValidClr(clr_prime))
				clr = clr_prime;
		}

		for (i = 0, p_i_node = chain.c_eef
			; i < i_end && NULL != p_i_node
			; i ++, p_i_node = p_i_node->GetParent())
		{
			p_i_node->SetColor(clr);
		}
		chain.SetColor(clr);
	}

}


class CIKGroupNodeGen 		// G
	: public CIKGroupNode
{
public:
	CIKGroupNodeGen(const CArtiBodyClrNode* jTree)
		: c_jTree(jTree)
	{
	}
	virtual void Dump(int n_indents) const override
	{
		std::stringstream logInfo;
		for (int i_indent = 0; i_indent < n_indents; i_indent ++)
			logInfo << "\t";
		logInfo << c_jTree->c_body->GetName_c()
				<< ": color = " << c_jTree->GetColor();
		logInfo << "{";
			for (auto chain : m_kChains)
			{
				chain->Dump(logInfo);
			}
		logInfo << "}";
		LOGIK(logInfo.str().c_str());
	}

public:
	const CArtiBodyClrNode* c_jTree;
};

class CIKGroupTreeGen
	: public CIKGroupTree
{
public:
	static CIKGroupNodeGen* Generate(const CArtiBodyClrNode* root);
	static void InitKChain(CIKGroupNodeGen* root, const std::vector<CIKChainClr>& chains);
};

CIKGroupNodeGen* CIKGroupTreeGen::Generate(const CArtiBodyClrNode* root)
{
	auto ConstructNodeGroupGen = [] (const CArtiBodyClrNode* src, CIKGroupNodeGen** dst) -> bool
								{
									const CArtiBodyClrNode* src_p = src->GetParent();
									bool new_group = ( (NULL == src_p)
													|| src->GetColor() != src_p->GetColor() );
									if (new_group)
									{
										*dst = new CIKGroupNodeGen(src);
									}
									return new_group;
								};
	CIKGroupNodeGen* root_G_gen = NULL;
	bool constructed = Construct(root, &root_G_gen, ConstructNodeGroupGen);
	LOGIKVar(LogInfoBool, constructed);
	IKAssert(constructed || NULL == root_G_gen);
	return root_G_gen;
}

void CIKGroupTreeGen::InitKChain(CIKGroupNodeGen* root, const std::vector<CIKChainClr>& chains)
{
	typedef std::list<CIKChainClr> GroupChains;
	std::vector<GroupChains> gid2gchains;
	const int clr_min = -1;
	const int clr_max = (int)chains.size();
	gid2gchains.resize(clr_max);
	for (const CIKChainClr& chain : chains)
	{
		int clr_i = chain.GetColor();
		IKAssert(clr_min < clr_i && clr_i < clr_max);
		gid2gchains[clr_i].push_back(chain);
	}

	const auto& c_gid2gchains = gid2gchains;

	auto OnGroupNode = [&c_gid2gchains] (CIKGroupNode* node_this)
				{
					CIKGroupNodeGen* node_this_gen = static_cast<CIKGroupNodeGen*>(node_this);
					int clr_this = node_this_gen->c_jTree->GetColor();
					if (!CArtiBodyClrTree::ValidClr(clr_this))
						return;
					const GroupChains& g_chains = c_gid2gchains[clr_this];
					for (const CIKChainClr& chain : g_chains)
					{
						node_this_gen->Joint(chain.Generate());
					}
				};

	auto OffGroupNode = [] (CIKGroupNode* node_this)
				{

				};
	CIKGroupTreeGen::TraverseDFS(root, OnGroupNode, OffGroupNode);
}

CIKGroupNode* CIKGroupTree::Generate(const CArtiBodyNode* root, const CONF::CBodyConf& ikChainConf)
{
	CIKGroupNode* root_G = NULL;
	CArtiBodyClrNode* root_clr = CArtiBodyClrTree::Generate(root);
	if (NULL != root_clr)
	{
		std::vector<CIKChainClr> chains;
		CArtiBodyClrTree::ColorGid(root_clr, ikChainConf, chains);
#ifdef _DEBUG
		CArtiBodyClrTree::Dump(root_clr);	// step 1
#endif
		if (root_clr->Colored())
		{
 			CIKGroupNodeGen* root_gen = CIKGroupTreeGen::Generate(root_clr);
			if (root_gen)
			{
#ifdef _DEBUG
 				CIKGroupTreeGen::Dump(root_gen);
				for (const CIKChainClr& clr : chains)
					clr.Dump();
#endif
 				CIKGroupTreeGen::InitKChain(root_gen, chains);
#ifdef _DEBUG
 				CIKGroupTreeGen::Dump(root_gen);
#endif
				auto GenerateNode = [](const CIKGroupNodeGen* src, CIKGroupNode** dst) -> bool
					{
						*dst = new CIKGroupNode(*src);
						return true;
					};
				bool constructed = CIKGroupTreeGen::Construct(root_gen, &root_G, GenerateNode);
				LOGIKVar(LogInfoBool, constructed);
				IKAssert(constructed || NULL == root_clr);
	 			CIKGroupTreeGen::Destroy(root_gen);
#ifdef _DEBUG
	 			CIKGroupTree::Dump(root_G);
#endif
			}
		}
		CArtiBodyClrTree::Destroy(root_clr);
	}
	return root_G;
}

#undef COLOR_BOTTOM