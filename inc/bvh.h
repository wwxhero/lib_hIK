#pragma once
#include "pch.h"
#ifdef __cplusplus
extern "C" {
#endif

HIKLIB(bool,			ResetRestPose)(const char* path_src, int frame, const char* path_dst);

#ifdef __cplusplus
};
#endif