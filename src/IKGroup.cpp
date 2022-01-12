#include "pch.h"
#include "IKGroup.hpp"

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
		group_origin->SetTranslation(g2w.getTranslation());;
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
	LOGIKVarErr(LogInfoCharPtr, m_rootBody->GetName_c());
	LOGIKVarErr(LogInfoBool, solved_all);
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

CIKGroup* CIKGroup::Clone() const
{
	return NULL;
}

CThreadIKGroup::CThreadIKGroup()
	: m_solved(false)
	, m_group(NULL)
{
}

CThreadIKGroup::~CThreadIKGroup()
{
	delete m_group;
}

void CThreadIKGroup::Initialize_main(const CIKGroup& group_src)
{
	m_group = group_src.Clone();
}

void CThreadIKGroup::Update_main(const Transform_TR& w2g, const TransformArchive& tm_0)
{
	m_solved = (m_group &&  !m_group->BeginUpdate(w2g, tm_0));
	Execute_main();
}

void CThreadIKGroup::Run_worker()
{
	if (!m_solved && m_group)
		m_solved = m_group->Update();
}

bool CThreadIKGroup::Solution_main(TransformArchive* tm_k)
{
	if (m_solved && m_group)
	{
		m_group->EndUpdate();
		CArtiBodyTree::Serialize<true>(m_group->RootBody(), *tm_k);
		return true;
	}
	else
		return false;
}

CIKGroupsParallel::CIKGroupsParallel()
{

}

CIKGroupsParallel::~CIKGroupsParallel()
{
}


void CIKGroupsParallel::Initialize(const CIKGroup& group_src, int n_concurrency)
{
	int thread_id = 0;
	m_pool.Initialize_main(n_concurrency,
						[&](CThreadIKGroup* thread)
							{
								thread->Initialize_main(group_src);
							});

}

void CIKGroupsParallel::Update_A(const Transform_TR& w2g, const TransformArchive& tm_0)
{
	auto thread_i = m_pool.WaitForAReadyThread_main(INFINITE);
	thread_i->Update_main(w2g, tm_0);
}

bool CIKGroupsParallel::Solution(TransformArchive* tm_star)
{
	bool solved = false;
	std::list<CThreadIKGroup*> readies;
	CThreadIKGroup* thread_i = NULL;
	while (!solved
			&& NULL !=
				(thread_i = m_pool.WaitForAReadyThread_main(0)))
	{
		readies.push_back(thread_i);
		solved = thread_i->Solution_main(tm_star);
	}

	for (auto thread_i : readies)
		thread_i->HoldReadyOn_main();
	return solved;
}