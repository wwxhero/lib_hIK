#pragma once
#include "pch.h"
#ifdef __cplusplus
extern "C" {
#endif

HIKLIB(bool,			ResetRestPose)(const char* path_src, int frame, const char* path_dst);
HIKLIB(HBODY,			create_tree_body_bvh)(const wchar_t* path_src);

#ifdef __cplusplus
};
#endif