#pragma once
#include "articulated_body.h"
#ifdef __cplusplus
extern "C" {
#endif

HIKLIB(HMOTIONNODE,		create_tree_motion_node)(HBODY mo_src);
HIKLIB(bool,			motion_sync_cnn_homo)(HMOTIONNODE parent, HMOTIONNODE child, enum CNN type);
HIKLIB(bool,			motion_sync_cnn_cross_c)(HMOTIONNODE parent, HMOTIONNODE child, enum CNN type, const char* pairs[][2], int n_pairs, Real p2c_w[3][4]);
HIKLIB(bool,			motion_sync_cnn_cross_w)(HMOTIONNODE parent, HMOTIONNODE child, enum CNN type, const wchar_t* pairs[][2], int n_pairs, Real p2c_w[3][4]);
HIKLIB(void,			motion_sync)(HMOTIONNODE root);
HIKLIB(void,			destroy_tree_motion_node)(HMOTIONNODE node);
HIKLIB(HMOTIONNODE,		get_first_child_mo_node)(HMOTIONNODE node);
HIKLIB(HMOTIONNODE,		get_next_sibling_mo_node)(HMOTIONNODE node);

#ifdef __cplusplus
}
#endif