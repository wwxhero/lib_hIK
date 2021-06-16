#pragma once
#include "pch.h"
#include "articulated_body.h"
#ifdef __cplusplus
extern "C" {
#endif

HIKLIB(HMOTIONNODE,		create_tree_motion_node)(HBODY mo_src);
HIKLIB(bool,			motion_sync_cnn_homo)(HMOTIONNODE parent, HMOTIONNODE child, enum CNN type, const char* pairs[][2], int n_pairs);
HIKLIB(bool,			motion_sync_cnn_cross)(HMOTIONNODE parent, HMOTIONNODE child, enum CNN type, const char* pairs[][2], int n_pairs);
HIKLIB(void,			motion_sync)(HMOTIONNODE root);
HIKLIB(void,			destroy_tree_motion_node)(HMOTIONNODE node);
HIKLIB(HMOTIONNODE,		get_first_child_mo_node)(HMOTIONNODE node);
HIKLIB(HMOTIONNODE,		get_next_sibling_mo_node)(HMOTIONNODE node);

#ifdef __cplusplus
}
#endif