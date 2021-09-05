#pragma once

#include <stack>
#include <queue>
#include "articulated_body.h"

template<typename TNode> class Tree;

template <typename This>
class TreeNode
{
	template<typename TNode>
	friend void Tree<TNode>::Connect(TNode* body_from, TNode* body_to, CNN type);
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
	This* GetParent() const
	{
		return m_parent;
	}

	void Dump(int indent) const
	{
	}

private:
	This* m_parent;
	This* m_firstChild;
	This* m_nextSibling;
};

template<typename NodeType>
class Tree
{
public:
	template<typename LAMaccessEnter, typename LAMaccessLeave>
	static void TraverseDFS(NodeType* root, LAMaccessEnter OnEnterBody, LAMaccessLeave OnLeaveBody)
	{
		assert(NULL != root);
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
			if (NULL == edge.body_child)
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

	template<typename LAMaccessEnter, typename LAMaccessLeave>
	static bool TraverseDFS(const NodeType* root, LAMaccessEnter OnEnterBody, LAMaccessLeave OnLeaveBody)
	{
		assert(NULL != root);
		typedef struct _EDGE
		{
			const NodeType* body_this;
			const NodeType* body_child;
		} EDGE;
		std::stack<EDGE> stkDFS;
		stkDFS.push({ root, root->GetFirstChild() });
		//printArtName(body_name_w(root), 0);
		bool walking = OnEnterBody(root);
		while (!stkDFS.empty()
			&& walking)
		{
			EDGE &edge = stkDFS.top();
			// size_t n_indent = stkDFS.size();
			if (NULL == edge.body_child)
			{
				stkDFS.pop();
				walking = OnLeaveBody(edge.body_this);
			}
			else
			{
				//printArtName(body_name_w(edge.body_child), n_indent);
				walking = OnEnterBody(edge.body_child);
				const NodeType* body_grandchild = edge.body_child->GetFirstChild();
				const NodeType* body_nextchild = edge.body_child->GetNextSibling();
				stkDFS.push({ edge.body_child, body_grandchild });
				edge.body_child = body_nextchild;
			}
		}
		return walking;
	}

	template<typename LAMaccess>
	static bool SearchBFS(const NodeType* root, LAMaccess OnSearchBody)
	{
		bool hit = false;
		std::queue<const NodeType*> bfs_q;
		bfs_q.push(root);
		while (!bfs_q.empty() && !hit)
		{
			const NodeType* node = bfs_q.front();
			bfs_q.pop();
			hit = OnSearchBody(node);
			for (auto child = node->GetFirstChild()
				; NULL != child && !hit
				; child = child->GetNextSibling())
				bfs_q.push(child);
		}
		return hit;
	}


	static void Connect(NodeType* body_from, NodeType* body_to, CNN type)
	{
		enum {forward = 0, inverse, total};
		NodeType** hook[total] = {NULL};
		NodeType*	target[total] = {NULL};
		if (CNN::FIRSTCHD == type)
		{
			body_to->m_nextSibling = body_from->m_firstChild;
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

	static void Destroy(NodeType* root)
	{
		auto onEnterBody = [](NodeType* node_this)
						{
						};

		auto onLeaveBody = [](NodeType* node_this)
						{
							delete node_this;
						};

		Tree<NodeType>::TraverseDFS(root, onEnterBody, onLeaveBody);
	}

	static void Dump(const NodeType* root)
	{
		int indent = 0;
		auto onEnterBody = [&indent](const NodeType* node_this) -> bool
						{
							indent ++;
							node_this->Dump(indent);
							return true;
						};

		auto onLeaveBody = [&indent](const NodeType* node_this) -> bool
						{
							indent --;
							return true;
						};

		Tree<NodeType>::TraverseDFS(root, onEnterBody, onLeaveBody);
	}

};