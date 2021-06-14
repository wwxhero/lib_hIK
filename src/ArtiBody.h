#pragma once
#include <string>
#include "Joint.h"
#include "Transform.h"
class CArtiBody
{
public:
	CArtiBody(const wchar_t *name
		, const _TRANSFORM* t_rest_local);
	CArtiBody(const char *name
		, const _TRANSFORM* t_rest_local);
	~CArtiBody();
	const wchar_t* GetName_w()
	{
		return m_namew.c_str();
	}
	const char* GetName_c()
	{
		return m_namec.c_str();
	}

	CArtiBody* GetFirstChild()
	{
		return m_firstChild;
	}
	CArtiBody* GetNextSibling()
	{
		return m_nextSibling;
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

	static void Connect(CArtiBody* body_from, CArtiBody* body_to, CNN type);
	static void FK_Update(CArtiBody* root);
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
	std::string m_namec;
	std::wstring m_namew;
	CArtiBody* m_parent;
	CArtiBody* m_firstChild;
	CArtiBody* m_nextSibling;
	//CJoint* m_joint;
	CTransform m_local2parent0;
	CTransform m_delta_l;

private:
	CTransform m_local2parent_cached;
	CTransform m_parent2local_cached;
	CTransform m_local2world_cached;
	CTransform m_world2local_cached;
};