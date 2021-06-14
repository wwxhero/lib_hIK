#include "pch.h"
#include <locale>
#include <codecvt>
#include "articulated_body.h"
#include "ArtiBody.h"

void CArtiBody::Connect(CArtiBody* body_from, CArtiBody* body_to, CNN type)
{
	enum {forward = 0, inverse, total};
	CArtiBody** hook[total] = {NULL};
	CArtiBody*	target[total] = {NULL};
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

void CArtiBody::GetTransformLocal2Parent(CTransform& l2p)
{
	l2p = m_local2parent0 * m_delta_l;
}

void CArtiBody::GetTransformLocal2World(CArtiBody* body, _TRANSFORM* tm_l2w)
{
	CTransform l2w;
	body->GetTransformLocal2Parent(l2w);
	for (CArtiBody* precceeding = body->m_parent
		; NULL != precceeding
		; precceeding = precceeding->m_parent)
	{
		CTransform l2p;
		precceeding->GetTransformLocal2Parent(l2p);
		l2w = l2p * l2w;
	}
	l2w.CopyTo(*tm_l2w);
}

void CArtiBody::GetJointTransform(CTransform& delta_l)
{
	delta_l = m_delta_l;
}

void CArtiBody::SetJointTransform(const CTransform& delta_l)
{
	m_delta_l = delta_l;
}

CArtiBody::CArtiBody(const wchar_t *name
					, const _TRANSFORM* tm_rest_l2p)
					: m_parent(NULL)
					, m_firstChild(NULL)
					, m_nextSibling(NULL)
					, m_local2parent0(*tm_rest_l2p)
{
	m_namew = name;
	std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
	m_namec = converter.to_bytes(m_namew);
}

CArtiBody::CArtiBody(const char *name
					, const _TRANSFORM* tm_rest_l2p)
					: m_parent(NULL)
					, m_firstChild(NULL)
					, m_nextSibling(NULL)
					, m_local2parent0(*tm_rest_l2p)
{
	m_namec = name;
	std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
	m_namew = converter.from_bytes(m_namec);
}

CArtiBody::~CArtiBody()
{
	m_parent = NULL;
	m_firstChild = NULL;
	m_nextSibling = NULL;
}

