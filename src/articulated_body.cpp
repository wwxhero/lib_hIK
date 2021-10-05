#include "pch.h"
#include "articulated_body.h"
#include <string>
#include <sstream>
#include <Windows.h>
#include "ArtiBody.hpp"
#include "handle_helper.hpp"
#include "ik_logger.h"
#include "matrix3.h"

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

bool clone_body_fbx(HBODY hSrc, HBODY* hDst)
{
	CArtiBodyNode* body_src = CAST_2PBODY(hSrc);
	CArtiBodyNode* body_dst = NULL;
	bool ret = CArtiBodyTree::Clone(body_src, &body_dst, CArtiBodyTree::CloneNode_fbx);
	if (ret)
		*hDst = CAST_2HBODY(body_dst);
	return ret;
}

bool clone_body_bvh(HBODY hSrc, HBODY* hDst)
{
	CArtiBodyNode* body_src = CAST_2PBODY(hSrc);
	CArtiBodyNode* body_dst = NULL;
	bool ret = CArtiBodyTree::Clone(body_src, &body_dst, CArtiBodyTree::CloneNode_bvh);
	if (ret)
		*hDst = CAST_2HBODY(body_dst);
	return ret;
}


bool clone_body_htr(HBODY hSrc, HBODY* hDst, const Real a_src2dst_w[3][3])
{
	CArtiBodyNode* body_src = CAST_2PBODY(hSrc);
	CArtiBodyNode* body_dst = NULL;

	Eigen::Matrix3r src2dst_w;
	src2dst_w <<
			a_src2dst_w[0][0], a_src2dst_w[0][1], a_src2dst_w[0][2],
			a_src2dst_w[1][0], a_src2dst_w[1][1], a_src2dst_w[1][2],
			a_src2dst_w[2][0], a_src2dst_w[2][1], a_src2dst_w[2][2];
	const auto& c_src2dst_w = src2dst_w;
#if 0
	bool verified = true;
	for (int i_r = 0; i_r < 3 && verified; i_r ++)
		for (int i_c = 0; i_c < 3 && verified; i_c ++)
			verified = (src2dst_w(i_r, i_c) == a_src2dst_w[i_r][i_c]);
	IKAssert(verified);
#endif

	auto CloneNode = [&c_src2dst_w] (const CArtiBodyNode* src, CArtiBodyNode** dst, const wchar_t* name_dst_opt) -> bool
					{
						return CArtiBodyTree::CloneNode_htr(src, dst, c_src2dst_w, name_dst_opt);
					};
	bool ret = CArtiBodyTree::Clone(body_src, &body_dst, CloneNode);
	if (ret)
		*hDst = CAST_2HBODY(body_dst);
	return ret;
}

bool clone_body_interests_htr(HBODY hSrc, HBODY* hDst, const wchar_t* (*matches)[2], int n_matches, bool src_on_match0, const Real a_src2dst_w[3][3])
{
	CArtiBodyNode* body_src = CAST_2PBODY(hSrc);
	CArtiBodyNode* body_dst = NULL;
	bool matches_not_specified = (NULL == matches || n_matches < 1);
	IKAssert(!matches_not_specified);

	Eigen::Matrix3r src2dst_w;
	src2dst_w <<
			a_src2dst_w[0][0], a_src2dst_w[0][1], a_src2dst_w[0][2],
			a_src2dst_w[1][0], a_src2dst_w[1][1], a_src2dst_w[1][2],
			a_src2dst_w[2][0], a_src2dst_w[2][1], a_src2dst_w[2][2];
#if 0
	bool verified = true;
	for (int i_r = 0; i_r < 3 && verified; i_r ++)
		for (int i_c = 0; i_c < 3 && verified; i_c ++)
			verified = (src2dst_w(i_r, i_c) == a_src2dst_w[i_r][i_c]);
	IKAssert(verified);
#endif
	bool ret = CArtiBodyTree::Clone_htr(body_src, &body_dst, matches, n_matches, src_on_match0, src2dst_w);
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

HBODY get_parent_body(HBODY body)
{
	CArtiBodyNode* artiBody = CAST_2PBODY(body);
	return CAST_2HBODY(artiBody->GetParent());
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

int body_cmp(const char* const pts_interest[], int n_interests, HBODY body_s, HBODY body_d, HBODY* err_nodes, Real* err_oris)
{
	CArtiBodyNode* artiBody_s = CAST_2PBODY(body_s);
	CArtiBodyNode* artiBody_d = CAST_2PBODY(body_d);
	return CArtiBodyTree::BodyCMP(pts_interest, n_interests, artiBody_s, artiBody_d, err_nodes, err_oris);
}

void body_T_test(HBODY body, const Real up[3]
			, const char* const pts_interest[], int n_interests
			, int part_body_idx_range[parts_total][2]
			, Real err[])
{
	CArtiBodyNode* artiBody = CAST_2PBODY(body);
	std::vector<std::string> pts(pts_interest, pts_interest + n_interests);
	Eigen::Vector3r dir_up(up[0], up[1], up[2]); dir_up.normalize();
	CArtiBodyTree::Body_T_Test(artiBody
							, dir_up
							, pts
							, part_body_idx_range
							, err);
}

HBODY* alloc_bodies(HBODY root, int *n_bodies)
{
	CArtiBodyNode* root_body = CAST_2PBODY(root);
	*n_bodies = 0;
	auto onEnterBody_Cnt = [n_bodies] (CArtiBodyNode *node)
		{
			(*n_bodies)++;
		};
	auto onLeaveBody_Cnt = [] (CArtiBodyNode *node)
		{
		};
	CArtiBodyTree::TraverseDFS(root_body, onEnterBody_Cnt, onLeaveBody_Cnt);

	HBODY* bodies = new HBODY[*n_bodies];
	int i_body = 0;
	auto onEnterBody_Assign = [&i_body, bodies] (CArtiBodyNode *node)
		{
			bodies[i_body ++] = CAST_2HBODY(node);
		};
	auto onLeaveBody_Assign = [] (CArtiBodyNode *node)
		{
		};
	CArtiBodyTree::TraverseDFS(root_body, onEnterBody_Assign, onLeaveBody_Assign);

	return bodies;
}

void free_bodies(HBODY* bodies)
{
	delete [] bodies;
}
