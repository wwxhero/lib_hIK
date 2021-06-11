#pragma once
#include "pch.h"

enum CNN {FIRSTCHD = 0, NEXTSIB};

struct _SCALE
{
	float x, y, z;
};

struct _ROT
{
	float w, x, y, z;
};

struct _TRANSLATE
{
	float x, y, z;
};

struct _TRANSFORM
{
	_SCALE s;
	_ROT r;
	_TRANSLATE tt;
};

#ifdef __cplusplus
extern "C" {
#endif

HIKLIB(float,			ik_test)(float theta);
HIKLIB(HBODY,			create_arti_body_f)(const wchar_t* name, const _TRANSFORM* tm_l2p);
HIKLIB(void,			destroy_arti_body)(HBODY hBody);
HIKLIB(void,			cnn_arti_body)(HBODY from, HBODY to, enum CNN);
HIKLIB(HBODY, 			get_first_child)(HBODY body);
HIKLIB(HBODY, 			get_next_sibling)(HBODY body);
HIKLIB(const wchar_t*, 	body_name_w)(HBODY body);
HIKLIB(void,			get_joint_transform_l2w)(HBODY body, _TRANSFORM* tm_l2w);


#ifdef __cplusplus
}
#endif