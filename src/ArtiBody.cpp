#include "pch.h"
#include <queue>
#include <stack>
#include "articulated_body.h"
#include "ArtiBody.h"


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

void CArtiBodyTree::FK_Update(CArtiBodyNode* root)
{
	for (auto body : root->m_kinalst)
		body->FK_UpdateNode();
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

void CArtiBodyNode::GetJointTransform(CTransform& delta_l)
{
	delta_l = m_delta_l;
}

void CArtiBodyNode::SetJointTransform(const CTransform& delta_l)
{
	m_delta_l = delta_l;
}

CArtiBodyNode::CArtiBodyNode(const wchar_t *name
					, const _TRANSFORM* tm_rest_l2p)
					: TreeNode<CArtiBodyNode>()
					, m_local2parent0(*tm_rest_l2p)
{
	m_namew = name;
	std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
	m_namec = converter.to_bytes(m_namew);
}

CArtiBodyNode::CArtiBodyNode(const char *name
					, const _TRANSFORM* tm_rest_l2p)
					: TreeNode<CArtiBodyNode>()
					, m_local2parent0(*tm_rest_l2p)
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

