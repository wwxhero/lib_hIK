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

HBODY create_arti_body_f(const wchar_t* name
						, const _TRANSFORM* t_rest_local)
{
	CArtiBody* body = new CArtiBody(name, t_rest_local);
	return body;
}

void destroy_arti_body(HBODY hBody)
{
	CArtiBody* body = reinterpret_cast<CArtiBody*>(hBody);
	delete body;
}

void cnn_arti_body(HBODY from, HBODY to, enum CNN type)
{
	CArtiBody* body_from = reinterpret_cast<CArtiBody*>(from);
	CArtiBody* body_to = reinterpret_cast<CArtiBody*>(to);
	CArtiBody::Connect(body_from, body_to, type);
}

const wchar_t* body_name_w(HBODY body)
{
	CArtiBody* artiBody = reinterpret_cast<CArtiBody*>(body);
	return artiBody->GetName_w();
}

HBODY get_first_child(HBODY body)
{
	CArtiBody* artiBody = reinterpret_cast<CArtiBody*>(body);
	return artiBody->GetFirstChild();
}

HBODY get_next_sibling(HBODY body)
{
	CArtiBody* artiBody = reinterpret_cast<CArtiBody*>(body);
	return artiBody->GetNextSibling();
}

void get_joint_transform_l2w(HBODY body, _TRANSFORM* tm_l2w)
{
	CArtiBody* artiBody = reinterpret_cast<CArtiBody*>(body);
	CArtiBody::GetJointTransformLocal2World(artiBody, tm_l2w);
}