#include "pch.h"
#include <queue>
#include <stack>
#include "articulated_body.h"
#include "ArtiBody.h"
#include "ik_logger.h"




CArtiBodyNode* CArtiBodyTree::CreateAnimNode(const wchar_t* name, const _TRANSFORM* tm)
{
	return CreateAnimNodeInternal(name, tm);
}

CArtiBodyNode* CArtiBodyTree::CreateAnimNode(const char* name, const _TRANSFORM* tm)
{
	return CreateAnimNodeInternal(name, tm);
}

CArtiBodyNode* CArtiBodyTree::CreateSimNode(const wchar_t* name, const _TRANSFORM* tm, TM_TYPE jtm)
{
	return CreateSimNodeInternal(name, tm, jtm);
}

CArtiBodyNode* CArtiBodyTree::CreateSimNode(const char* name, const _TRANSFORM* tm, TM_TYPE jtm)
{
	return CreateSimNodeInternal(name, tm, jtm);
}

void CArtiBodyTree::FK_Update(CArtiBodyNode* root)
{
	if (anim == root->c_type)
	{
		for (auto body : root->m_kinalst)
			static_cast<CArtiBodyNode_anim*>(body)->FK_UpdateNode();
	}
	else // sim == root->c_type
	{
		for (auto body : root->m_kinalst)
		{
			switch (body->c_jtmflag)
			{
				case t_r:
					static_cast<CArtiBodyNode_sim_r*>(body)->FK_UpdateNode();
					break;
				case t_tr:
					static_cast<CArtiBodyNode_sim_tr*>(body)->FK_UpdateNode();
					break;
			}
		}
	}
}

void CArtiBodyTree::KINA_Initialize(CArtiBodyNode* root)
{
	auto onEnterBody = [](CArtiBodyNode* node_this)
					{
						node_this->m_kinalst.clear();
						node_this->m_kinalst.push_back(node_this);
					};

	auto onLeaveBody = [](CArtiBodyNode* node_this)
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

	Tree<CArtiBodyNode>::TraverseDFS_botree_nonrecur(root, onEnterBody, onLeaveBody);
}

void CArtiBodyTree::Destroy(CArtiBodyNode* root)
{
	auto onEnterBody = [](CArtiBodyNode* node_this)
					{
					};

	auto onLeaveBody = [](CArtiBodyNode* node_this)
					{
						delete node_this;
					};

	Tree<CArtiBodyNode>::TraverseDFS_botree_nonrecur(root, onEnterBody, onLeaveBody);
}

#ifdef _DEBUG
void CArtiBodyTree::Connect(CArtiBodyNode* from, CArtiBodyNode* to, CNN type)
{
	assert(from->c_type == to->c_type);
	Super::Connect(from, to, type);
}
#endif

CArtiBodyNode::CArtiBodyNode(const wchar_t *name, NODETYPE type, TM_TYPE jtmflag)
					: TreeNode<CArtiBodyNode>()
					, c_type(type)
					, c_jtmflag(jtmflag)
{
	m_namew = name;
	std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
	m_namec = converter.to_bytes(m_namew);
}

CArtiBodyNode::CArtiBodyNode(const char *name, NODETYPE type, TM_TYPE jtmflag)
					: TreeNode<CArtiBodyNode>()
					, c_type(type)
					, c_jtmflag(jtmflag)
{
	m_namec = name;
	std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
	m_namew = converter.from_bytes(m_namec);
}

CArtiBodyNode::~CArtiBodyNode()
{
	m_parent = NULL;
	m_firstChild = NULL;
	m_nextSibling = NULL;
}

