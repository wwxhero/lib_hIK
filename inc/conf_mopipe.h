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


	HIKLIB(HCONFMOPIPE,		init_conf_mopipe)(HCONF conf);
	HIKLIB(void,			uninit_conf_mopipe)(HCONFMOPIPE conf);
	HIKLIB(bool,			load_scale)(HCONFMOPIPE conf, B_Scale* bscales[2], int n_scales[2]);
	HIKLIB(void,			free_scale)(B_Scale* bscales[2], int n_scales[2]);
	HIKLIB(bool,			load_endeff_names)(HCONFMOPIPE conf, const wchar_t** names[2], int n_names[2]);
	HIKLIB(void,			free_endeff_names)(const wchar_t** names[2], int n_names[2]);
	HIKLIB(bool,			get_mopipe_mtx)(HCONFMOPIPE conf, Real m[3][3]);
	HIKLIB(int,				load_mopipe_pairs)(HCONFMOPIPE conf, const wchar_t* (**match)[2]);
	HIKLIB(void,			free_mopipe_pairs)(const wchar_t* (*match)[2], int n_match);
	HIKLIB(bool,			load_file_names)(HCONFMOPIPE conf, const wchar_t* filenames[2]);
	HIKLIB(void,			free_file_names)(const wchar_t* filenames[2]);


#ifdef __cplusplus
};
#endif