#pragma once
#include <string>
#include <list>
#include <stack>
#include "Joint.h"
#include "Transform.h"

template <typename This>
class TreeNode
{
	template<typename TNode>
	friend class Tree;
protected:
	TreeNode()
		: m_parent(NULL)
		, m_firstChild(NULL)
		, m_nextSibling(NULL)
	{}
public:
	This* GetFirstChild() const
	{
		return m_firstChild;
	}
	This* GetNextSibling() const
	{
		return m_nextSibling;
	}

protected:
	This* m_parent;
	This* m_firstChild;
	This* m_nextSibling;
};

template<typename NodeType>
class Tree
{
public:
	template<typename LAMaccessEnter, typename LAMaccessLeave>
	static void TraverseDFS_botree_nonrecur(NodeType* root, LAMaccessEnter OnEnterBody, LAMaccessLeave OnLeaveBody)
	{
		assert(H_INVALID != root);
		typedef struct _EDGE
		{
			NodeType* body_this;
			NodeType* body_child;
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
				NodeType* body_grandchild = edge.body_child->GetFirstChild();
				NodeType* body_nextchild = edge.body_child->GetNextSibling();
				stkDFS.push({ edge.body_child, body_grandchild });
				edge.body_child = body_nextchild;
			}
		}
	}


	static void Connect(NodeType* body_from, NodeType* body_to, CNN type)
	{
		enum {forward = 0, inverse, total};
		NodeType** hook[total] = {NULL};
		NodeType*	target[total] = {NULL};
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


};

class CArtiBodyNode : public TreeNode<CArtiBodyNode>
{
	friend class CArtiBodyTree;
public:
	CArtiBodyNode(const wchar_t *name
		, const _TRANSFORM* t_rest_local);
	CArtiBodyNode(const char *name
		, const _TRANSFORM* t_rest_local);
	~CArtiBodyNode();
	const wchar_t* GetName_w()
	{
		return m_namew.c_str();
	}
	const char* GetName_c()
	{
		return m_namec.c_str();
	}

	void GetTransformLocal2Parent(CTransform& l2p)
	{
		l2p = m_local2parent_cached;
	}

	void GetTransformLocal2World(CTransform& l2w)
	{
		l2w = m_local2world_cached;
	}

	void GetJointTransform(CTransform& delta);
	void SetJointTransform(const CTransform& delta);

private:
	inline void FK_UpdateNode()
	{
		//todo: update cached transformations
		m_local2parent_cached = m_local2parent0 * m_delta_l;
		m_parent2local_cached = m_local2parent_cached.inverse();

		m_local2world_cached = (NULL == m_parent
								? m_local2parent_cached
								: m_parent->m_local2world_cached * m_local2parent_cached);
		// m_world2local_cached = m_parent2local_cached * m_parent->m_world2local_cached;
		m_world2local_cached = m_local2world_cached.inverse();
	}
private:
	std::list<CArtiBodyNode*> m_kinalst;
private:
	std::string m_namec;
	std::wstring m_namew;
	//CJoint* m_joint;
	CTransform m_local2parent0;
	CTransform m_delta_l;

private:
	CTransform m_local2parent_cached;
	CTransform m_parent2local_cached;
	CTransform m_local2world_cached;
	CTransform m_world2local_cached;
};

class CArtiBodyTree : Tree<CArtiBodyNode>
{
public:
	static void KINA_Initialize(CArtiBodyNode* root);
	static void FK_Update(CArtiBodyNode* root);
};