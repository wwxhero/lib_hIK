#include "pch.h"
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

CArtiBody::CArtiBody(const wchar_t *name
					, const _TRANSFORM* t_rest_local)
					: m_parent(NULL)
					, m_firstChild(NULL)
					, m_nextSibling(NULL)
{
	m_namew = name;
}

CArtiBody::~CArtiBody()
{
	m_parent = NULL;
	m_firstChild = NULL;
	m_nextSibling = NULL;
}

