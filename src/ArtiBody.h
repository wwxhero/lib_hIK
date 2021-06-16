#pragma once
#include <string>
#include <list>
#include "TreeBase.h"
#include "Joint.h"
#include "Transform.h"



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