#pragma once
#include "pch.h"

enum CNN
{
	FIRSTCHD = 0,
	NEXTSIB
};

enum NODETYPE
{
	anim,
	sim
};

struct _SCALE
{
	Real x, y, z;
};

struct _ROT
{
	Real w, x, y, z;
};

struct _TRANSLATE
{
	Real x, y, z;
};

struct _TRANSFORM
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
};


#ifdef __cplusplus
extern "C" {
#endif

HIKLIB(float,			ik_test)(float theta);
HIKLIB(HBODY,			create_anim_body_node_w)(const wchar_t* name, const _TRANSFORM* tm_l2p);
HIKLIB(HBODY,			create_anim_body_node_c)(const char* name, const _TRANSFORM* tm_l2p);
HIKLIB(HBODY,			create_sim_body_node_w)(const wchar_t* name, const _TRANSFORM* tm_l2p, TM_TYPE jtm);
HIKLIB(HBODY,			create_sim_body_node_c)(const char* name, const _TRANSFORM* tm_l2p, TM_TYPE jtm);
HIKLIB(void,			destroy_tree_body_node)(HBODY hBody);
HIKLIB(void,			destroy_tree_body)(HBODY hBody);
HIKLIB(void,			cnn_arti_body)(HBODY from, HBODY to, enum CNN);
HIKLIB(HBODY,			get_first_child_body)(HBODY body);
HIKLIB(HBODY,			get_next_sibling_body)(HBODY body);
HIKLIB(const wchar_t*,	body_name_w)(HBODY body);
HIKLIB(const char*,		body_name_c)(HBODY body);
HIKLIB(void,			get_body_transform_l2w)(HBODY body, _TRANSFORM* tm_l2w);
HIKLIB(void,			get_body_transform_l2p)(HBODY body, _TRANSFORM* tm_l2w);
HIKLIB(void,			log_body_node)(HBODY body);



#ifdef __cplusplus
}
#endif