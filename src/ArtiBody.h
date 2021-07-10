#pragma once
#include <string>
#include <list>
#include "TreeBase.h"
#include "Joint.h"
#include "Transform.h"



class CArtiBodyNode : public TreeNode<CArtiBodyNode>
{
	friend class CArtiBodyTree;
	friend class CMoNode;		//for accessing m_kinalst
public:
	CArtiBodyNode(const wchar_t *name
		, const _TRANSFORM* t_rest_local);
	CArtiBodyNode(const char *name
		, const _TRANSFORM* t_rest_local);
	~CArtiBodyNode();
	const wchar_t* GetName_w() const
	{
		return m_namew.c_str();
	}
	const char* GetName_c() const
	{
		return m_namec.c_str();
	}

	void GetTransformLocal2Parent(Transform_TRS& l2p) const
	{
		l2p = m_local2parent_cached;
	}

	void GetTransformParent2Local(Transform_TRS& p2l) const
	{
		p2l = m_parent2local_cached;
	}

	void GetTransformLocal2World(Transform_TRS& l2w) const
	{
		l2w = m_local2world_cached;
	}

	void GetTransformWorld2Local(Transform_TRS& w2l) const
	{
		w2l = m_world2local_cached;
	}

	void GetJointTransform(Transform_TRS& delta);
	void SetJointTransform(const Transform_TRS& delta);

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
	Transform_TRS m_local2parent0;
	Transform_TRS m_delta_l;

private:
	Transform_TRS m_local2parent_cached;
	Transform_TRS m_parent2local_cached;
	Transform_TRS m_local2world_cached;
	Transform_TRS m_world2local_cached;
};

class CArtiBodyTree : Tree<CArtiBodyNode>
{
public:
	static void KINA_Initialize(CArtiBodyNode* root);
	static void FK_Update(CArtiBodyNode* root);
	static void Destroy(CArtiBodyNode* root);
};