#include "pch.h"
#include "fk_joint.h"
#include "Joint.hpp"
#include "ArtiBody.hpp"
#include "handle_helper.hpp"
#include "ik_logger.h"

void set_joint_transform(HBODY body, const _TRANSFORM* tm_l)
{
	//todo: assign transform delta to the articulated body joint
	CArtiBodyNode* artiBody = CAST_2PBODY(body);
	artiBody->GetJoint()->GetTransform()->CopyFrom(*tm_l);
}

void get_joint_transform(HBODY body, _TRANSFORM* tm_l)
{
	CArtiBodyNode* artiBody = CAST_2PBODY(body);
	const Transform* tm = artiBody->GetJoint()->GetTransform();
	tm->CopyTo(*tm_l);
}

void initialize_kina(HBODY root)
{
	CArtiBodyNode* artiBody = CAST_2PBODY(root);
	CArtiBodyTree::KINA_Initialize(artiBody);
}

void update_fk(HBODY root)
{
	CArtiBodyNode* artiBody = CAST_2PBODY(root);
	CArtiBodyTree::FK_Update<false>(artiBody);
}

