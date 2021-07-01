#pragma once
#include "pch.h"
#ifdef __cplusplus
extern "C" {
#endif



HIKLIB(bool,			ResetRestPose)(const char* path_src, int frame, const char* path_dst);
HIKLIB(HBVH,			load_bvh)(const wchar_t* path_src);
HIKLIB(unsigned int,	get_n_frames)(HBVH bvh);
HIKLIB(void,			unload_bvh)(HBVH bvh);
HIKLIB(HBODY,			create_tree_body_bvh)(HBVH bvh);



#ifdef __cplusplus
};
#endif