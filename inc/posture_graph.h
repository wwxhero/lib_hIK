#pragma once
#include "pch.h"
#ifdef __cplusplus
extern "C" {
#endif

HIKLIB(void, err_vis)(const char* path_htr, const char* path_png);
HIKLIB(void, dissect)(const char* confXML, const char* path_htr, const char* dir_out);

#ifdef __cplusplus
};
#endif