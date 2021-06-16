#include "pch.h"
#include <queue>
#include <stack>
#include <locale>
#include <codecvt>
#include "articulated_body.h"
#include "ArtiBody.h"

void CArtiBody::Connect(CArtiBody* body_from, CArtiBody* body_to, CNN type)
{
	enum {forward = 0, inverse, total};
	CArtiBody** hook[total] = {NULL};
	CArtiBody*	target[total] = {NULL};
	if (CNN::FIRSTCHD == type)
	{
		hook[forward] = &body_from->m_firstChild;
		target[forward] = body_to;
		hook[inverse] = &body_to->m_parent;
		target[inverse] = body_from;
	}
	else
	{
		hook[forward] = &body_from->m_nextSibling;
		target[forward] = body_to;
		hook[inverse] = &body_to->m_parent;
		target[inverse] = body_from->m_parent;
	}

	*hook[forward] = target[forward];
	*hook[inverse] = target[inverse];
}

template<typename LAMaccessEnter, typename LAMaccessLeave>
inline void TraverseDFS_botree_nonrecur(CArtiBody* root, LAMaccessEnter OnEnterBody, LAMaccessLeave OnLeaveBody)
{
	assert(H_INVALID != root);
	typedef struct _EDGE
	{
		CArtiBody* body_this;
		CArtiBody* body_child;
	} EDGE;
	std::stack<EDGE> stkDFS;
	stkDFS.push({ root, root->GetFirstChild() });
	//printArtName(body_name_w(root), 0);
	OnEnterBody(root);
	while (!stkDFS.empty())
	{
		EDGE &edge = stkDFS.top();
		// size_t n_indent = stkDFS.size();
		if (H_INVALID == edge.body_child)
		{
			stkDFS.pop();
			OnLeaveBody(edge.body_this);
		}
		else
		{
			//printArtName(body_name_w(edge.body_child), n_indent);
			OnEnterBody(edge.body_child);
			CArtiBody* body_grandchild = edge.body_child->GetFirstChild();
			CArtiBody* body_nextchild = edge.body_child->GetNextSibling();
			stkDFS.push({ edge.body_child, body_grandchild });
			edge.body_child = body_nextchild;
		}
	}
}

void CArtiBody::KINA_Initialize(CArtiBody* root)
{
	auto onEnterBody = [](CArtiBody* node_this)
					{
						node_this->m_kinalst.clear();
						node_this->m_kinalst.push_back(node_this);
					};

	auto onLeaveBody = [](CArtiBody* node_this)
					{
						auto& this_kinalst = node_this->m_kinalst;
						for (auto node_child = node_this->GetFirstChild();
							node_child != NULL;
							node_child = node_child->GetNextSibling())
						{
							const auto& child_kinalst = node_child->m_kinalst;
							this_kinalst.insert(this_kinalst.end(),
												child_kinalst.begin(),
												child_kinalst.end());
						}
					};

	TraverseDFS_botree_nonrecur(root, onEnterBody, onLeaveBody);
}

void CArtiBody::FK_Update(CArtiBody* root)
{
	for (auto body : root->m_kinalst)
		body->FK_UpdateNode();
}

void CArtiBody::GetJointTransform(CTransform& delta_l)
{
	delta_l = m_delta_l;
}

void CArtiBody::SetJointTransform(const CTransform& delta_l)
{
	m_delta_l = delta_l;
}

CArtiBody::CArtiBody(const wchar_t *name
					, const _TRANSFORM* tm_rest_l2p)
					: m_parent(NULL)
					, m_firstChild(NULL)
					, m_nextSibling(NULL)
					, m_local2parent0(*tm_rest_l2p)
{
	m_namew = name;
	std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
	m_namec = converter.to_bytes(m_namew);
}

CArtiBody::CArtiBody(const char *name
					, const _TRANSFORM* tm_rest_l2p)
					: m_parent(NULL)
					, m_firstChild(NULL)
					, m_nextSibling(NULL)
					, m_local2parent0(*tm_rest_l2p)
{
	m_namec = name;
	std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
	m_namew = converter.from_bytes(m_namec);
}

CArtiBody::~CArtiBody()
{
	m_parent = NULL;
	m_firstChild = NULL;
	m_nextSibling = NULL;
}

