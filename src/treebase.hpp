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

	virtual void Dump(int indent) const
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

	template<typename TSrc, typename TDst, typename LAMaccess>
	static bool ConstructBFS(const TSrc* root_src, TDst** a_root_dst, LAMaccess ConstructNode)
	{
		typedef std::pair<const TSrc*, TDst*> Bound;
		std::queue<Bound> queBFS;
		TDst* root_dst = NULL;
		bool root_constructed = ConstructNode(root_src, &root_dst);
		if (root_constructed)
		{
			Bound root = std::make_pair(
				root_src,
				root_dst
			);
			queBFS.push(root);
			while (!queBFS.empty())
			{
				Bound pair = queBFS.front();
				const TSrc* body_src = pair.first;
				TDst* body_dst = pair.second;
				CNN cnn = FIRSTCHD;
				TDst* b_this_dst = body_dst;
				for (const TSrc* child_body_src = (const TSrc*)body_src->GetFirstChild()
					; NULL != child_body_src
					; child_body_src = (const TSrc*)child_body_src->GetNextSibling())
				{
					TDst* child_body_dst = NULL;
					bool constructed = ConstructNode(child_body_src, &child_body_dst);
					if (constructed)
					{
						Bound child = std::make_pair(
							child_body_src,
							child_body_dst
						);
						queBFS.push(child);
						TDst* b_next_dst = child_body_dst;
						Connect(b_this_dst, b_next_dst, cnn);
						cnn = NEXTSIB;
						b_this_dst = b_next_dst;
					}
					else
					{
						Bound child = std::make_pair(
							child_body_src,
							body_dst
						);
						queBFS.push(child);
					}
				}
				queBFS.pop();
			}
		}

		if (root_constructed)
		{
			*a_root_dst = root_dst;
		}
		else
		{
			if (NULL != root_dst)
				Destroy(root_dst);
			*a_root_dst = NULL;
		}
		return root_constructed;
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
			body_to->m_nextSibling = body_from->m_nextSibling;
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

	template<typename LAMBDA_nodecmp, typename LAMBDA_onBound>
	static bool TraverseBFS_Bound(const NodeType* root_s, const NodeType* root_d, LAMBDA_nodecmp nodeCmp, LAMBDA_onBound onBound)
	{
		std::queue<const NodeType*> que_s;
		std::queue<const NodeType*> que_d;
		que_s.push(root_s);
		que_d.push(root_d);
		bool traversing = true;
		while ((traversing = (traversing && (que_s.empty() == que_d.empty())))
			&& !que_s.empty())
		{
			auto node_s = que_s.front();
			auto node_d = que_d.front();
			traversing = onBound(node_s, node_d);
			std::list<const NodeType*> children_s;
			std::list<const NodeType*> children_d;
			const NodeType *child_s, *child_d;
			for (child_s = node_s->GetFirstChild(), child_d = node_d->GetFirstChild()
				; (traversing = (traversing && ((NULL == child_s) == (NULL == child_d))))
					&& NULL != child_s
				; child_s = child_s->GetNextSibling(), child_d = child_d->GetNextSibling())
			{
				children_s.push_back(child_s);
				children_d.push_back(child_d);
			}
			children_s.sort(nodeCmp);
			children_d.sort(nodeCmp);
			for (auto child_s : children_s)
				que_s.push(child_s);
			for (auto child_d : children_d)
				que_d.push(child_d);
			que_s.pop();
			que_d.pop();
		}
		return traversing;
	}
};