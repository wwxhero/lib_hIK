#pragma once
#include "pch.h"
#ifdef __cplusplus
extern "C" {
#endif

HIKLIB(bool, err_vis)(const char* interests_conf, const char* path_htr, const char* path_png);
HIKLIB(bool, dissect)(const char* confXML, const char* path_htr, const char* dir_out);
HIKLIB(bool, trim)(const char* src, const char* dst, const char* const names_rm[], int n_names);
HIKLIB(bool, posture_graph_gen)(const char* interests_conf_path, const char* path_htr, const char* dir_out, Real epsErr);
HIKLIB(bool, convert_pg2dot)(const char* path_src, const char* path_dst);

#ifdef __cplusplus
};
#endif