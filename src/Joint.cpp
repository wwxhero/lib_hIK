#include "pch.h"
#include "fk_joint.h"
#include "Joint.h"
#include "ArtiBody.h"


void set_joint_transform(HBODY body, const _TRANSFORM* tm_l)
{
	//todo: assign transform delta to the articulated body joint
	CArtiBodyNode* artiBody = reinterpret_cast<CArtiBodyNode*>(body);
	CTransform tm(*tm_l);
	artiBody->SetJointTransform(tm);
}

void get_joint_transform(HBODY body, _TRANSFORM* tm_l)
{
	CArtiBodyNode* artiBody = reinterpret_cast<CArtiBodyNode*>(body);
	CTransform tm;
	artiBody->GetJointTransform(tm);
	tm.CopyTo(*tm_l);
}

void initialize_kina(HBODY root)
{
	CArtiBodyNode* artiBody = reinterpret_cast<CArtiBodyNode*>(root);
	CArtiBodyTree::KINA_Initialize(artiBody);
}

void update_fk(HBODY root)
{
	CArtiBodyNode* artiBody = reinterpret_cast<CArtiBodyNode*>(root);
	CArtiBodyTree::FK_Update(artiBody);
}