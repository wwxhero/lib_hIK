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

bool CIKGroup::BeginUpdate()
{
	int n_chains = (int)m_kChains.size();
	if (n_chains < 1)
		return false;
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