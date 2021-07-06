#include "pch.h"
#include "Transform.h"

CTransform CTransform::Scale(Real s)
{
	CTransform tm;
	tm.m_t.linear() = tm.m_t.linear() * s;
	return tm;
}