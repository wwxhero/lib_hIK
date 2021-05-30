#pragma once
#include <string>
class CArtiBody
{
public:
	CArtiBody(const wchar_t *name
		, const _TRANSFORM* t_rest_local);
	~CArtiBody();
	const wchar_t* GetName_w()
	{
		return m_namew.c_str();
	}
	CArtiBody* GetFirstChild()
	{
		return m_firstChild;
	}
	CArtiBody* GetNextSibling()
	{
		return m_nextSibling;
	}

	static void Connect(CArtiBody* body_from, CArtiBody* body_to, CNN type);
private:
	std::string m_name;
	std::wstring m_namew;
	CArtiBody* m_parent;
	CArtiBody* m_firstChild;
	CArtiBody* m_nextSibling;
};