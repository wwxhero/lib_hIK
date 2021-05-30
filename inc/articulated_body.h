#pragma once
#include "pch.h"
#define H_INVALID NULL
typedef void* HBODY;

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

HIKAPI(float,			ik_test)(float theta);
HIKAPI(HBODY,			create_arti_body_f)(const wchar_t* name, const _TRANSFORM* t_rest_local);
HIKAPI(void,			cnn_arti_body)(HBODY from, HBODY to, enum CNN);
HIKAPI(HBODY, 			get_first_child)(HBODY body);
HIKAPI(HBODY, 			get_next_sibling)(HBODY body);
HIKAPI(const wchar_t*, 	body_name_w)(HBODY body);

#ifdef __cplusplus
}
#endif