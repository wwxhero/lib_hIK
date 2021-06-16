#pragma once

#include <stack>
#include "articulated_body.h"

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