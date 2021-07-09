#include "pch.h"
#include "Transform.h"

Affine3 Affine3::Scale(Real s)
{
	Affine3 tm;
	tm.linear() = tm.linear() * s;
	return tm;
}