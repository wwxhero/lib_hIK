#include "pch.h"
#include "fk_joint.h"
#include "Joint.h"
#include "ArtiBody.h"


void set_joint_transform(HBODY body, const _TRANSFORM* tm_l)
{
	//todo: assign transform delta to the articulated body joint
	CArtiBody* artiBody = reinterpret_cast<CArtiBody*>(body);
	CTransform tm(*tm_l);
	artiBody->SetJointTransform(tm);
}

void get_joint_transform(HBODY body, _TRANSFORM* tm_l)
{
	CArtiBody* artiBody = reinterpret_cast<CArtiBody*>(body);
	CTransform tm;
	artiBody->GetJointTransform(tm);
	tm.CopyTo(*tm_l);
}
