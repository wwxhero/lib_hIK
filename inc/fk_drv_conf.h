#pragma once
#include "pch.h"
#include "conf.h"
#ifdef __cplusplus
extern "C" {
#endif

	typedef struct _B_Scale
	{
		const wchar_t* bone_name;
		float scaleX;
		float scaleY;
		float scaleZ;
	} B_Scale;


	HIKLIB(HCONFFKRC,		init_fkrc)(HCONF conf);
	HIKLIB(void,			uninit_fkrc)(HCONFFKRC conf);
	HIKLIB(int,				load_rc_scale)(HCONFFKRC conf, B_Scale** bscales);
	HIKLIB(void,			free_rc_scale)(B_Scale* bscales, int n_scales);
	HIKLIB(bool,			get_mopipe_mtx)(HCONFFKRC conf, Real m[3][3]);
	HIKLIB(int,				load_mopipe_pairs)(HCONFFKRC conf, const wchar_t* (**match)[2]);
	HIKLIB(void,			free_mopipe_pairs)(const wchar_t* (*match)[2], int n_match);



#ifdef __cplusplus
};
#endif