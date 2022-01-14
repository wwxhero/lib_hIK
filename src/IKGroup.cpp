#include "pch.h"
#include "IKGroup.hpp"
#include "IKChainInverseJK.hpp"


CIKGroup::CIKGroup(CArtiBodyNode* root)
	: m_rootBody(root)
{
}

CIKGroup::CIKGroup(CIKGroup& src)
{
	m_rootBody = src.m_rootBody;
	m_kChains = std::move(src.m_kChains);
}

CIKGroup::~CIKGroup()
{
	for (auto chain : m_kChains)
		delete chain;
}

void CIKGroup::SetupTargets(const std::map<std::wstring, CArtiBodyNode*>& nameSrc2bodyDst
								, const Eigen::Matrix3r& src2dst_w
								, const Eigen::Matrix3r& dst2src_w)
{
	for (auto chain : m_kChains)
		chain->SetupTarget(nameSrc2bodyDst, src2dst_w, dst2src_w);
}

bool CIKGroup::BeginUpdate(Transform_TR* w2g)
{
	int n_chains = (int)m_kChains.size();
	IKAssert(n_chains > 0);
	Transform_TR tm_w2g_tr;
	CArtiBodyNode* g_parent = m_rootBody->GetParent();
	if (NULL != g_parent)
	{
		const Transform* tm_w2g = g_parent->GetTransformWorld2Local();
		IKAssert(t_tr == tm_w2g->Type());
		tm_w2g_tr = *static_cast<const Transform_TR*>(tm_w2g);
	}

	bool exist_an_update = false;
	for (int i_chain = 0; i_chain < n_chains; i_chain ++)
	{
		bool updating_i = m_kChains[i_chain]->BeginUpdate(tm_w2g_tr);
		exist_an_update = (exist_an_update || updating_i);
	}
	*w2g = tm_w2g_tr;
	return exist_an_update;
}

bool CIKGroup::BeginUpdate(const Transform_TR& tm_w2g_tr, const TransformArchive& tm_0)
{
	CArtiBodyNode* g_parent = m_rootBody->GetParent();
	if (NULL != g_parent)
	{
		Transform_TR g2w = tm_w2g_tr.inverse();
		IJoint* group_origin = g_parent->GetJoint();
		group_origin->SetRotation(g2w.getRotation_q());
		group_origin->SetTranslation(g2w.getTranslation());
	}

	CArtiBodyTree::Serialize<false>(m_rootBody, const_cast<TransformArchive&>(tm_0));
	CArtiBodyTree::FK_Update<false>(m_rootBody);

	int n_chains = (int)m_kChains.size();
	IKAssert(n_chains > 0);
	bool exist_an_update = false;
	for (int i_chain = 0; i_chain < n_chains; i_chain ++)
	{
		bool updating_i = m_kChains[i_chain]->BeginUpdate(tm_w2g_tr);
		exist_an_update = (exist_an_update || updating_i);
	}

	return exist_an_update;
}

bool CIKGroup::Update()
{
	int n_chains = (int)m_kChains.size();
	bool solved_all = false;

	if (1 == n_chains)
	{
		solved_all = m_kChains[0]->Update();
	}
	else
	{
		for (int i_update = 0; i_update < n_chains && !solved_all; i_update ++)
		{
			for (auto& chain_i : m_kChains)
				chain_i->Update();

			solved_all = true;
			for (auto chain_i = m_kChains.begin()
				; solved_all && chain_i != m_kChains.end()
				; chain_i ++)
				solved_all = (*chain_i)->UpdateCompleted();
		}
	}
	// LOGIKVarErr(LogInfoCharPtr, m_rootBody->GetName_c());
	// LOGIKVarErr(LogInfoBool, solved_all);
	return solved_all;
}

void CIKGroup::EndUpdate()
{
	int n_chains = (int)m_kChains.size();
	for (int i_chain = 0; i_chain < n_chains; i_chain++)
	{
		m_kChains[i_chain]->EndUpdate();
	}
	CArtiBodyTree::FK_Update<false>(m_rootBody);
}

void CIKGroup::IKReset()
{
	for (auto chain_i : m_kChains)
		chain_i->Reset();
	CArtiBodyTree::FK_Update<false>(m_rootBody);
}

void CIKGroup::Dump(int n_indents) const
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

void CIKGroup::Dump(int n_indents, std::ostream& logInfo) const
{
	for (int i_indent = 0; i_indent < n_indents; i_indent++)
		logInfo << "\t";
	logInfo << "{";
	for (auto chain : m_kChains)
	{
		chain->Dump(logInfo);
	}
	logInfo << "}";
}

CIKChain* CIKGroup::AddChain(const CONF::CIKChainConf* conf)
{
	const CArtiBodyNode* eef = NULL;
	auto SearchOnBody = [&eef, conf](const CArtiBodyNode* node) -> bool
		{
			if (std::string(node->GetName_c()) == conf->eef)
			{
				eef = node;
				return true;
			}
			else
				return false;
		};

	CArtiBodyTree::SearchBFS(m_rootBody, SearchOnBody);

	if (NULL == eef)
		return NULL;

	CIKChain* chain = NULL;
	switch(conf->algor)
	{
		case CIKChain::Proj:
			chain = new CIKChainProj(conf->up);
			break;
		case CIKChain::DLS:
			chain = new CIKChainInverseJK_DLS(conf->weight_p
											, conf->weight_r
											, conf->n_iter);
			break;
		case CIKChain::SDLS:
			chain = new CIKChainInverseJK_SDLS(conf->weight_p
											, conf->weight_r
											, conf->n_iter);
			break;
	}

	if (!chain->Init(eef, conf->len, conf->Joints))
	{
		delete chain;
		chain = NULL;
	}
	else
		Join(chain);
	return chain;
}

CThreadIKGroup::CThreadIKGroup()
	: m_solved(false)
	, m_group(NULL)
	, m_rootParent(NULL)
{
}

CThreadIKGroup::~CThreadIKGroup()
{
	delete m_group;
	CArtiBodyTree::Destroy(m_rootParent);
}

void CThreadIKGroup::Initialize_main(const CArtiBodyNode* root_src)
{
	//fixme: create an IKGroup with a duplicated ArtiBody from the root
	const CArtiBodyNode* root_parent_src = root_src->GetParent();
	CArtiBodyNode* root_dup = NULL;
	if (root_parent_src)
		CArtiBodyTree::Clone(root_parent_src, &m_rootParent, true);
	else
		CArtiBodyTree::Clone(root_src, &m_rootParent);

	std::string root_name(root_src->GetName_c());
	auto SearchOnBody = [&root_dup, &root_name = std::as_const(root_name)](const CArtiBodyNode* node) -> bool
		{
			if (std::string(node->GetName_c()) == root_name)
			{
				root_dup = const_cast<CArtiBodyNode*>(node);
				return true;
			}
			else
				return false;
		};

	CArtiBodyTree::SearchBFS(m_rootParent, SearchOnBody);

	IKAssert(NULL != root_dup);

	m_group = new CIKGroup(root_dup);
}

void CThreadIKGroup::Update_main(const Transform_TR& w2g, const TransformArchive& tm_0)
{
	m_solved = (m_group &&  !m_group->BeginUpdate(w2g, tm_0));
	Execute_main();
}

CIKChain* CThreadIKGroup::AddChain_main(const CONF::CIKChainConf* conf)
{
	if (m_group)
		return m_group->AddChain(conf);
	else
		return NULL;
}

void CThreadIKGroup::SetupTargets_main(const std::map<std::wstring, CArtiBodyNode*>& nameSrc2bodyDst, const Eigen::Matrix3r& src2dst_w, const Eigen::Matrix3r& dst2src_w)
{
	if (m_group)
	{
		m_group->SetupTargets(nameSrc2bodyDst, src2dst_w, dst2src_w);
	}
}

void CThreadIKGroup::Run_worker()
{
	if (!m_solved && m_group)
		m_solved = m_group->Update();
}

bool CThreadIKGroup::AcqUpdateRes_main(TransformArchive* tm_k)
{
	if (m_group && m_solved)
	{
		m_group->EndUpdate();
		CArtiBodyTree::Serialize<true>(m_group->RootBody(), *tm_k);
		m_solved = false; // reset m_solved for the subsequent IK tasks
		return true;
	}
	else
		return false;
}

CIKGroupsParallel::CIKGroupsParallel(const CArtiBodyNode* root, int concurrency)
{
	m_pool.Initialize_main(concurrency,
						[&](CThreadIKGroup* thread)
							{
								thread->Initialize_main(root);
							});
}

CIKGroupsParallel::CIKGroupsParallel(CIKGroupsParallel& src)
	: m_pool(std::move(src.m_pool))
{
}

CIKGroupsParallel::~CIKGroupsParallel()
{
}

bool CIKGroupsParallel::Update_A(const Transform_TR& w2g, TransformArchive& tm_0)
{
	auto thread_i = m_pool.WaitForAReadyThread_main(INFINITE);
	TransformArchive& tm_k = tm_0;
	if (thread_i->AcqUpdateRes_main(&tm_k))
	{
		thread_i->HoldReadyOn_main();
		return true;
	}
	else
	{
		thread_i->Update_main(w2g, tm_0);
		return false;
	}
}

bool CIKGroupsParallel::SolutionFinal(TransformArchive* tm_star)
{
	bool solved = false;

	auto& threads = m_pool.WaitForAllReadyThreads_main();
	for (auto it = threads.begin()
		; it != threads.end() && !solved
		; it ++)
		solved = (*it)->AcqUpdateRes_main(tm_star);

	for (auto thread_i : threads)
		thread_i->HoldReadyOn_main();

	return solved;
}

void CIKGroupsParallel::AddChain(const CONF::CIKChainConf* conf)
{
	auto threads = m_pool.WaitForAllReadyThreads_main();
	for (auto thread_i : threads)
	{
		thread_i->AddChain_main(conf);
		thread_i->HoldReadyOn_main();
	}
}

void CIKGroupsParallel::SetupTargets(const std::map<std::wstring, CArtiBodyNode*>& nameSrc2bodyDst, const Eigen::Matrix3r& src2dst_w, const Eigen::Matrix3r& dst2src_w)
{
	auto threads = m_pool.WaitForAllReadyThreads_main();
	for (auto thread_i : threads)
	{
		thread_i->SetupTargets_main(nameSrc2bodyDst, src2dst_w, dst2src_w);
		thread_i->HoldReadyOn_main();
	}
}