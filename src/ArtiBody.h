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

	void GetJointTransformLocal2Parent(CTransform& l2p);

	static void Connect(CArtiBody* body_from, CArtiBody* body_to, CNN type);
	static void GetJointTransformLocal2World(CArtiBody* body, _TRANSFORM* tm_l2w);
private:
	std::string m_namec;
	std::wstring m_namew;
	CArtiBody* m_parent;
	CArtiBody* m_firstChild;
	CArtiBody* m_nextSibling;
	//CJoint* m_joint;
	CTransform m_local2parent;
};