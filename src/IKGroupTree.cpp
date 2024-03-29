#include "pch.h"
#include "IKGroupTree.hpp"
#include "IKChainInverseJK.hpp"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CIKGroupNode:

CIKGroupNode::CIKGroupNode(CArtiBodyNode* root, int concurrency, int attempts)
	: m_primary(root)
	, m_secondary(root, concurrency)
	, c_restartAttempts(attempts)
	, m_pg(NULL)
{

}

CIKGroupNode::CIKGroupNode(CIKGroupNode& src)
	: m_primary(src.m_primary)
	, m_secondary(src.m_secondary)
	, c_restartAttempts(src.c_restartAttempts)
{
	m_pg = src.m_pg; src.m_pg = NULL;
}

CIKGroupNode::~CIKGroupNode()
{
	if (NULL != m_pg)
		delete m_pg;
}


void CIKGroupNode::SetupTargets(const std::map<std::wstring, CArtiBodyNode*>& nameSrc2bodyDst
								, const Eigen::Matrix3r& src2dst_w
								, const Eigen::Matrix3r& dst2src_w)
{
	m_primary.SetupTargets(nameSrc2bodyDst, src2dst_w, dst2src_w);
	m_secondary.SetupTargets(nameSrc2bodyDst, src2dst_w, dst2src_w);
}

void CIKGroupNode::LoadPostureGraph(const char* pgDir, int radius)
{
	if (!m_primary.Empty())
	{
		m_pg = new CPGRuntimeParallel();
		if (!m_pg->Load(pgDir, m_primary.RootBody(), radius))
		{
			delete m_pg;
			m_pg = NULL;
		}
		else
		{
			m_pg->SetActivePosture<false>(0, true);
		}
	}
}

CIKChain* CIKGroupNode::AddChain(const CONF::CIKChainConf* chainConf)
{
	CIKChain* ret = m_primary.AddChain(chainConf);
	m_secondary.AddChain(chainConf);
	return ret;
}

void CIKGroupNode::IKUpdate()
{
	if (m_primary.Empty())
		return;

	// LOGIKErr("BeginPrimaryUpdate");
	Transform_TR w2g;
	if (!m_primary.BeginUpdate(&w2g))
	{
		// LOGIKErr("EndPrimaryUpdate");
		return;
	}
	auto root_body = m_primary.RootBody();

	if (m_pg)
		m_pg->ApplyActivePosture<true>();  		// apply active posture with respect to group space
	else
		m_primary.IKReset<true>();				// for the group which has no posture graph, searching from the binding posture

	bool updated = m_primary.Update();

	// if (updated)
		m_primary.EndUpdate();

	// LOGIKErr("EndPrimaryUpdate");
	if (m_pg)
	{
		if (!updated)
		{
			CArtiBodyTree::Serialize<true>(root_body, m_tmk0);
			m_secondary.BeginUpdate(w2g);

			CPGRuntimeParallel::Locker locker;
			auto pg_seq = m_pg->Lock(&locker);

			int n_errs = 0;
			int n_localMinima = 0;

			auto IKErr = [&](int pose_id, bool* stop_searching) -> Real
				{
					n_errs ++;
					*stop_searching = (updated || n_errs > m_pg->Radius() || n_localMinima > c_restartAttempts);
					if (*stop_searching)
					{
						// LOGIKVarErr(LogInfoBool, updated);
						return std::numeric_limits<Real>::max();
					}
					else
					{
						pg_seq->SetActivePosture<true>(pose_id, true);
						Real err = m_primary.Error();
						// LOGIKVarErr(LogInfoReal, err);
						return err;
					}
				};

			auto OnPG_Lomin = [&](int pose_id)
				{
					n_localMinima ++;
					pg_seq->SetActivePosture<true>(pose_id, true);
					CArtiBodyTree::Serialize<true>(root_body, m_tmk);
					updated = m_secondary.Update_A(m_tmk);
				};

			CPGRuntime::LocalMin(*pg_seq, IKErr, OnPG_Lomin);
			m_pg->UnLock(locker);

			if (updated)
			{
				m_secondary.EndUpdate(m_tmk0, &m_tmk);
				CArtiBodyTree::Serialize<false>(root_body, m_tmk);
			}
			else
				CArtiBodyTree::Serialize<false>(root_body, m_tmk0);

			// LOGIKVarErr(LogInfoInt, n_errs);
			// LOGIKVarErr(LogInfoInt, n_localMinima);

			// LOGIKErr("EndSecondaryUpdate");
		}
		m_pg->UpdateFKProj();
	}
	// LOGIKVarErr(LogInfoCharPtr, root_body->GetName_c());
	// LOGIKVarErr(LogInfoBool, updated);
	CArtiBodyTree::FK_Update<false>(root_body);


}

void CIKGroupNode::IKReset()
{
	if (m_pg)
		m_pg->SetActivePosture<false>(0, false);
	m_primary.IKReset<false>();
}

void CIKGroupNode::Dump(int n_indents) const
{
	m_primary.Dump(n_indents);
}

void CIKGroupNode::Dump(int n_indents, std::ostream& logInfo) const
{
	m_primary.Dump(n_indents, logInfo);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CArtiBodyClrNode
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CIKChainClr

class CIKChainClr
{
public:
	CIKChainClr(CArtiBodyClrNode* eef, const CONF::CIKChainConf& a_conf, int clr, int dist2root)
		: c_eef(eef)
		, c_conf(&a_conf)
		, m_Color(clr)
		, c_dist2root(dist2root)
	{
	}

	CIKChainClr(const CIKChainClr& src)
		: m_Color(src.m_Color)
		, c_eef(src.c_eef)
		, c_conf(src.c_conf)
		, c_dist2root(src.c_dist2root)
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
		LOGIKVar(LogInfoInt, c_conf->len);
		LOGIKVar(LogInfoInt, c_dist2root);
	}

public:
	int m_Color;
	CArtiBodyClrNode* c_eef;
	const CONF::CIKChainConf* c_conf;
	int c_dist2root;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CArtiBodyClrTree

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
	bool constructed = ConstructBFS(root, &root_clr, GenerateNode);
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

	auto OffNode = [&bodyConf, &chains, &dist2root] (CArtiBodyClrNode* node_this)
				{
					dist2root--;
					int i_eef = bodyConf.Name2IKChainIdx(node_this->c_body->GetName_c());
					if (i_eef > -1)
					{
						CIKChainClr chain(node_this, bodyConf.IK_Chains[i_eef], i_eef, dist2root);
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
		int n = chain.c_conf->len;
		int i_end = n;
		CArtiBodyClrNode* p_i_node = NULL;
		int i = 0;
		for (i = 0, p_i_node = chain.c_eef->GetParent()
			; i < i_end && NULL != p_i_node
			; i ++, p_i_node = p_i_node->GetParent())
		{
			int clr_prime = p_i_node->GetColor();
			if (CArtiBodyClrTree::ValidClr(clr_prime))
				clr = clr_prime;
		}

		chain.c_eef->SetColor(clr);		// for 0 length IK chain, eef must be colored
		for (i = 0, p_i_node = chain.c_eef->GetParent()
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
	CIKGroupNodeGen(const CArtiBodyClrNode* jTree, int concurrency, int attempts)
		: CIKGroupNode(const_cast<CArtiBodyNode*>(jTree->c_body), concurrency, attempts)
		, c_jTree(jTree)
	{
	}
	virtual void Dump(int n_indents) const override
	{
		std::stringstream logInfo;
		for (int i_indent = 0; i_indent < n_indents; i_indent ++)
			logInfo << "\t";
		logInfo << c_jTree->c_body->GetName_c()
				<< ": color = " << c_jTree->GetColor();
		LOGIK(logInfo.str().c_str());
		CIKGroupNode::Dump(n_indents);
	}

public:
	const CArtiBodyClrNode* c_jTree;
};

class CIKGroupTreeGen
	: public CIKGroupTree
{
public:
	static CIKGroupNodeGen* Generate(const CArtiBodyClrNode* root, int concur, int attempts);
	static void InitKChain(CIKGroupNodeGen* root, const std::vector<CIKChainClr>& chains);
};

CIKGroupNodeGen* CIKGroupTreeGen::Generate(const CArtiBodyClrNode* root, int concur, int attempts)
{
	auto ConstructNodeGroupGen = [concur, attempts] (const CArtiBodyClrNode* src, CIKGroupNodeGen** dst) -> bool
								{
									const CArtiBodyClrNode* src_p = src->GetParent();
									bool new_group = ( (NULL == src_p)
													|| src->GetColor() != src_p->GetColor() );
									if (new_group)
									{
										*dst = new CIKGroupNodeGen(src, concur, attempts);
									}
									return new_group;
								};
	CIKGroupNodeGen* root_G_gen = NULL;
	bool constructed = ConstructBFS(root, &root_G_gen, ConstructNodeGroupGen);
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
					for (const CIKChainClr& chainclr : g_chains)
					{
						if (NULL == node_this_gen->AddChain(chainclr.c_conf))
						{
							std::stringstream err;
							err << "Chain: " << chainclr.c_eef->c_body->GetName_c() << " failed initialization!!!";
							LOGIKVarErr(LogInfoCharPtr, err.str().c_str());
						}

					}
				};

	auto OffGroupNode = [] (CIKGroupNode* node_this)
				{

				};
	CIKGroupTreeGen::TraverseDFS(root, OnGroupNode, OffGroupNode);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// CIKGroupTree

CIKGroupNode* CIKGroupTree::Generate(const CArtiBodyNode* root, const CONF::CBodyConf& ikChainConf)
{
	CIKGroupNode* root_G = NULL;
	CArtiBodyClrNode* root_clr = CArtiBodyClrTree::Generate(root);
	if (NULL != root_clr)
	{
		std::vector<CIKChainClr> chains;
		CArtiBodyClrTree::ColorGid(root_clr, ikChainConf, chains);
#if defined _DEBUG || defined SMOOTH_LOGGING
		CArtiBodyClrTree::Dump(root_clr);	// step 1
#endif
		if (root_clr->Colored()
			|| (ikChainConf.Name2IKChainIdx(root->GetName_c()) > -1))	//root node is either colored or an end effector
		{
 			CIKGroupNodeGen* root_gen = CIKGroupTreeGen::Generate(root_clr
																, ikChainConf.PG_restart_concurrency()
																, ikChainConf.PG_restart_attempts());
			if (root_gen)
			{
#if defined _DEBUG || defined SMOOTH_LOGGING
 				CIKGroupTreeGen::Dump(root_gen);
				for (const CIKChainClr& clr : chains)
					clr.Dump();
#endif
 				CIKGroupTreeGen::InitKChain(root_gen, chains);
#if defined _DEBUG || defined SMOOTH_LOGGING
 				CIKGroupTreeGen::Dump(root_gen);
#endif
				auto GenerateNode = [](const CIKGroupNodeGen* src, CIKGroupNode** dst) -> bool
					{
						*dst = new CIKGroupNode(*(const_cast<CIKGroupNodeGen*>(src)));
						return true;
					};
				bool constructed = CIKGroupTreeGen::ConstructBFS(root_gen, &root_G, GenerateNode);
				LOGIKVar(LogInfoBool, constructed);
				IKAssert(constructed || NULL == root_clr);
	 			CIKGroupTreeGen::Destroy(root_gen);
#if defined _DEBUG || defined SMOOTH_LOGGING
	 			CIKGroupTree::Dump(root_G);
#endif
			}
		}
#if defined _DEBUG|| defined SMOOTH_LOGGING
		LOGIKFlush();
#endif
		CArtiBodyClrTree::Destroy(root_clr);
	}
	return root_G;
}

void CIKGroupTree::SetupTargets(CIKGroupNode* root_ik
								, const std::map<std::wstring, CArtiBodyNode*>& nameSrc2bodyDst
								, const Eigen::Matrix3r& src2dst_w
								, const Eigen::Matrix3r& dst2src_w)
{
	auto OnIKGroupNode = [&nameSrc2bodyDst, &src2dst_w, &dst2src_w](CIKGroupNode* gNode)
		{
			gNode->SetupTargets(nameSrc2bodyDst, src2dst_w, dst2src_w);
		};

	auto OffIKGroupNode = [](CIKGroupNode* gNode)
		{

		};

	CIKGroupTree::TraverseDFS(root_ik, OnIKGroupNode, OffIKGroupNode);
}

void CIKGroupTree::LoadPG(CIKGroupNode* root_ik, const char* dirPath, int radius)
{
	auto OnIKGroupNode = [dirPath, radius](CIKGroupNode* gNode)
		{
			gNode->LoadPostureGraph(dirPath, radius);
		};

	auto OffIKGroupNode = [](CIKGroupNode* gNode)
		{

		};

	CIKGroupTree::TraverseDFS(root_ik, OnIKGroupNode, OffIKGroupNode);
}

#undef COLOR_BOTTOM
