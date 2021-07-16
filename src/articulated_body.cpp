#include "pch.h"
#include "articulated_body.h"
#include <string>
#include <sstream>
#include <Windows.h>
#include "ArtiBody.h"
#include "handle_helper.hpp"
#include "ik_logger.h"

float ik_test(float theta)
{
	//std::stringstream info;
	//info << theta;
	//MessageBoxA(NULL, info.str().c_str(), NULL, MB_OK);
	return ++ theta;
}

HBODY create_fbx_body_node_w(const wchar_t* name
						, const _TRANSFORM* t_rest_local)
{
	CArtiBodyNode* body = CArtiBodyTree::CreateAnimNode(name, t_rest_local);
	return CAST_2HBODY(body);
}

HBODY create_fbx_body_node_c(const char* name
						, const _TRANSFORM* t_rest_local)
{
	CArtiBodyNode* body = CArtiBodyTree::CreateAnimNode(name, t_rest_local);
	return CAST_2HBODY(body);
}

HBODY create_bvh_body_node_w(const wchar_t* name
						, const _TRANSFORM* t_rest_local
						, TM_TYPE jtm)
{
	CArtiBodyNode* body = CArtiBodyTree::CreateSimNode(name, t_rest_local, bvh, jtm);
	return CAST_2HBODY(body);
}

HBODY create_bvh_body_node_c(const char* name
						, const _TRANSFORM* t_rest_local
						, TM_TYPE jtm)
{
	CArtiBodyNode* body = CArtiBodyTree::CreateSimNode(name, t_rest_local, bvh, jtm);
	return CAST_2HBODY(body);
}

bool clone_body(HBODY hSrc, BODY_TYPE nodetype, HBODY* hDst)
{
	CArtiBodyNode* body_src = CAST_2PBODY(hSrc);
	CArtiBodyNode* body_dst = NULL;
	bool ret = CArtiBodyTree::Clone(body_src, nodetype, &body_dst);
	if (ret)
		*hDst = CAST_2HBODY(body_dst);
	return ret;
}

void destroy_tree_body_node(HBODY hBody)
{
	CArtiBodyNode* body = CAST_2PBODY(hBody);
	delete body;
}

void destroy_tree_body(HBODY hBody)
{
	CArtiBodyNode* body = CAST_2PBODY(hBody);
	CArtiBodyTree::Destroy(body);
}

void cnn_arti_body(HBODY from, HBODY to, enum CNN type)
{
	CArtiBodyNode* body_from = CAST_2PBODY(from);
	CArtiBodyNode* body_to = CAST_2PBODY(to);
	CArtiBodyTree::Connect(body_from, body_to, type);
}

const wchar_t* body_name_w(HBODY body)
{
	CArtiBodyNode* artiBody = CAST_2PBODY(body);
	return artiBody->GetName_w();
}

const char* body_name_c(HBODY body)
{
	CArtiBodyNode* artiBody = CAST_2PBODY(body);
	return artiBody->GetName_c();
}

HBODY get_first_child_body(HBODY body)
{
	CArtiBodyNode* artiBody = CAST_2PBODY(body);
	return CAST_2HBODY(artiBody->GetFirstChild());
}

HBODY get_next_sibling_body(HBODY body)
{
	CArtiBodyNode* artiBody = CAST_2PBODY(body);
	return CAST_2HBODY(artiBody->GetNextSibling());
}

void get_body_transform_l2w(HBODY body, _TRANSFORM* tm_l2w)
{
	CArtiBodyNode* artiBody = CAST_2PBODY(body);
	const Transform* tm = artiBody->GetTransformLocal2World();
	tm->CopyTo(*tm_l2w);
}

void get_body_transform_l2p(HBODY body, _TRANSFORM* tm_l2w)
{
	CArtiBodyNode* artiBody = CAST_2PBODY(body);
	const Transform* tm = artiBody->GetTransformLocal2Parent();
	tm->CopyTo(*tm_l2w);
}

void log_body_node(HBODY body)
{
	std::stringstream logInfo;
	CArtiBodyNode* artiBody = CAST_2PBODY(body);
	const Transform* tm = artiBody->GetTransformLocal2Parent();
	logInfo << artiBody->GetName_c() << ":" << tm->ToString().c_str();
	LOGIK(logInfo.str().c_str());
}
