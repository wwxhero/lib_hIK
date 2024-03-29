#pragma once
#include "pch.h"
#ifdef __cplusplus
extern "C" {
#endif

struct _ERROR_TB
{
	Real* data;
	int n_rows;
	int n_cols;
};

HIKLIB(bool, init_err_tb)(const char* interests_conf, const char* path_htr, _ERROR_TB* err_tb);
HIKLIB(bool, init_err_tb_merged)(const char* interests_conf, const char* pg_theta_0, const char* pg_theta_1, _ERROR_TB* err_tb);
HIKLIB(void, uninit_err_tb)(_ERROR_TB* err_tb);
HIKLIB(Real, err_entry)(const _ERROR_TB* err_tb, int i_row, int i_col);
HIKLIB(bool, dissect)(const char* confXML, const char* path_htr, const char* dir_out);
HIKLIB(bool, trim)(const char* src, const char* dst, const char* const names_rm[], int n_names);
HIKLIB(bool, posture_graph_gen)(const char* interests_conf_path, const char* path_htr, const char* dir_out, Real epsErr, int* n_theta_raw, int* n_theta_pg);
HIKLIB(HPG, posture_graph_load)(const char* pg_dir_0, const char* pg_name);
HIKLIB(void, posture_graph_release)(HPG hPG);
HIKLIB(HPG, posture_graph_merge)(HPG pg_0, HPG pg_1, const char* confXML, Real eps_err); // hg = hg_0 U hg_1
HIKLIB(bool, posture_graph_save)(HPG hpg, const char* dir_out);
HIKLIB(bool, convert_pg2dot)(const char* path_src, const char* path_dst);
HIKLIB(int, N_Theta)(HPG pg);
HIKLIB(bool, remove_theta_noise)(const char* path_src, const char* path_dst, const char* path_interests_conf);
HIKLIB(bool, extract_joint_rotation)(const char* path_interests_conf, const char* path_src, const char* path_dst_rot, const char* path_dst_lim);


#ifdef __cplusplus
};
#endif