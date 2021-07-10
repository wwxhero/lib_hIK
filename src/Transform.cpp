#include "pch.h"
#include "Transform.h"

Transform_TRS Transform_TRS::Scale(Real s)
{
	Transform_TRS tm;
	tm.linear() = tm.linear() * s;
	return tm;
}