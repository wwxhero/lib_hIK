#pragma once
#include "pch.h"
#ifdef __cplusplus
extern "C" {
#endif

	HIKLIB(HCONF,			load_conf)(const wchar_t* path_src);
	HIKLIB(void,			unload_conf)(HCONF conf);

#ifdef __cplusplus
};
#endif