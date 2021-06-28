#include "pch.h"
#include "articulated_body.h"
#include <string>
#include <sstream>
#include <Windows.h>
#include "ArtiBody.h"


float ik_test(float theta)
{
	//std::stringstream info;
	//info << theta;
	//MessageBoxA(NULL, info.str().c_str(), NULL, MB_OK);
	return -- theta;
}

HBODY create_tree_body_node_w(const wchar_t* name
						, const _TRANSFORM* t_rest_local)
{
	CArtiBodyNode* body = new CArtiBodyNode(name, t_rest_local);
	return body;
}

HBODY create_tree_body_node_c(const char* name
						, const _TRANSFORM* t_rest_local)
{
	CArtiBodyNode* body = new CArtiBodyNode(name, t_rest_local);
	return body;
}

void destroy_tree_body_node(HBODY hBody)
{
	CArtiBodyNode* body = reinterpret_cast<CArtiBodyNode*>(hBody);
	delete body;
}

void destroy_tree_body(HBODY hBody)
{
	CArtiBodyNode* body = reinterpret_cast<CArtiBodyNode*>(hBody);
	CArtiBodyTree::Destroy(body);
}

void cnn_arti_body(HBODY from, HBODY to, enum CNN type)
{
	CArtiBodyNode* body_from = reinterpret_cast<CArtiBodyNode*>(from);
	CArtiBodyNode* body_to = reinterpret_cast<CArtiBodyNode*>(to);
	Tree<CArtiBodyNode>::Connect(body_from, body_to, type);
}

const wchar_t* body_name_w(HBODY body)
{
	CArtiBodyNode* artiBody = reinterpret_cast<CArtiBodyNode*>(body);
	return artiBody->GetName_w();
}

const char* body_name_c(HBODY body)
{
	CArtiBodyNode* artiBody = reinterpret_cast<CArtiBodyNode*>(body);
	return artiBody->GetName_c();
}

HBODY get_first_child_body(HBODY body)
{
	CArtiBodyNode* artiBody = reinterpret_cast<CArtiBodyNode*>(body);
	return artiBody->GetFirstChild();
}

HBODY get_next_sibling_body(HBODY body)
{
	CArtiBodyNode* artiBody = reinterpret_cast<CArtiBodyNode*>(body);
	return artiBody->GetNextSibling();
}

void get_body_transform_l2w(HBODY body, _TRANSFORM* tm_l2w)
{
	CArtiBodyNode* artiBody = reinterpret_cast<CArtiBodyNode*>(body);
	CTransform tm;
	artiBody->GetTransformLocal2World(tm);
	tm.CopyTo(*tm_l2w);
}

void get_body_transform_l2p(HBODY body, _TRANSFORM* tm_l2w)
{
	CArtiBodyNode* artiBody = reinterpret_cast<CArtiBodyNode*>(body);
	CTransform tm;
	artiBody->GetTransformLocal2Parent(tm);
	tm.CopyTo(*tm_l2w);
}

