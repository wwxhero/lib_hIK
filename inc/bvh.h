#pragma once
#include "pch.h"
#ifdef __cplusplus
extern "C" {
#endif



HIKLIB(bool,			ResetRestPose)(const char* path_src, int frame, const char* path_dst);
HIKLIB(HBVH,			load_bvh_w)(const wchar_t* path_src);
HIKLIB(HBVH,			load_bvh_c)(const char* path_src);
HIKLIB(unsigned int,	get_n_frames)(HBVH bvh);
HIKLIB(void,			unload_bvh)(HBVH bvh);
HIKLIB(HBODY,			create_tree_body_bvh)(HBVH bvh);
HIKLIB(void,			pose_body)(HBVH bvh, HBODY body, int i_frame);
HIKLIB(unsigned int,	channels)(HBVH bvh);
HIKLIB(unsigned int,	frames)(HBVH bvh);
HIKLIB(double,			frame_time)(HBVH bvh);
HIKLIB(void,			PrintJointHierarchy)(HBVH hBVH);



#ifdef __cplusplus
};
#endif