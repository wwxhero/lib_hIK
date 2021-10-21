#pragma once
#include "pch.h"
#ifdef __cplusplus
extern "C" {
#endif

HIKLIB(bool, err_vis)(const char* interests_conf, const char* path_htr, const char* path_png);
HIKLIB(bool, dissect)(const char* confXML, const char* path_htr, const char* dir_out);
HIKLIB(bool, posture_graph_gen)(const char* path_htr, const char* dir_out);

#ifdef __cplusplus
};
#endif