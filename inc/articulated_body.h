#pragma once
#include "pch.h"

enum CNN
{
	FIRSTCHD = 0,
	NEXTSIB
};

enum BODY_TYPE
{
	undef = 0,
	fbx = 0x00000001,
	bvh = 0x00000002,
	htr = 0x00000004,
	anim = fbx,
	sim = bvh|htr,
};

struct _SCALE
{
	Real x, y, z;
};

struct alignas(16) _ROT
{
	Real w, x, y, z;
};

struct _TRANSLATE
{
	Real x, y, z;
};

struct alignas(16) _TRANSFORM  //ocuppies 40 bytes
{
	_SCALE s;
	_ROT r;
	_TRANSLATE tt;
};

enum TM_TYPE {
	t_tt = 0x00000001,
	t_rx = 0x00000002,
	t_ry = 0x00000004,
	t_rz = 0x00000008,
	t_r  = t_rx|t_ry|t_rz,
	t_tr = t_tt|t_r,
	t_s	 = 0x00000010,
	t_trs = t_tt|t_r|t_s,
	t_none = 0
};

enum PART {
	spine = 0,
	right_leg,
	left_leg,
	right_arm,
	left_arm,
	parts_total
};


#ifdef __cplusplus
extern "C" {
#endif

HIKLIB(HBODY,			create_fbx_body_node_w)(const wchar_t* name, const _TRANSFORM* tm_l2p);
HIKLIB(HBODY,			create_fbx_body_node_c)(const char* name, const _TRANSFORM* tm_l2p);
HIKLIB(HBODY,			create_bvh_body_node_w)(const wchar_t* name, const _TRANSFORM* tm_l2p, TM_TYPE jtm);
HIKLIB(HBODY,			create_bvh_body_node_c)(const char* name, const _TRANSFORM*  tm_l2p, TM_TYPE jtm);
HIKLIB(bool,			clone_body_fbx)(HBODY src, HBODY* dst);
HIKLIB(bool,			clone_body_bvh)(HBODY src, HBODY* dst);
HIKLIB(bool,			clone_body_htr)(HBODY src, HBODY* dst, const Real src2dst_w[3][3]);
HIKLIB(bool,			clone_body_interests_htr)(HBODY src, HBODY* dst, const wchar_t* (*matches)[2], int n_matches, bool src_on_match0, const Real src2dst_w[3][3]);
HIKLIB(void,			destroy_tree_body_node)(HBODY hBody);
HIKLIB(void,			destroy_tree_body)(HBODY hBody);
HIKLIB(void,			cnn_arti_body)(HBODY from, HBODY to, enum CNN);
HIKLIB(HBODY,			get_parent_body)(HBODY body);
HIKLIB(HBODY,			get_first_child_body)(HBODY body);
HIKLIB(HBODY,			get_next_sibling_body)(HBODY body);
HIKLIB(const wchar_t*,	body_name_w)(HBODY body);
HIKLIB(const char*,		body_name_c)(HBODY body);
HIKLIB(void,			get_body_transform_l2w)(HBODY body, _TRANSFORM* tm_l2w);
HIKLIB(void,			get_body_transform_l2p)(HBODY body, _TRANSFORM* tm_l2w);

HIKLIB(void,			log_body_node)(HBODY body);
HIKLIB(void,			body_T_test)(HBODY body, const Real up[3], const Real forward[3]
								, const char* const pts_interest[], int n_interests
								, int part_idx_range[parts_total][2]
								, Real err[]);
HIKLIB(void,			body_EQ_test)(HBODY body_s, HBODY body_d, const char* const pts_interest[], int n_interests
								, Real err[]);
HIKLIB(HBODY*,			alloc_bodies)(HBODY root, int *n_bodies);
HIKLIB(void,			free_bodies)(HBODY* bodies);

#ifdef __cplusplus
}
#endif