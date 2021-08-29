#pragma once
#include "articulated_body.h"
#ifdef __cplusplus
extern "C" {
#endif

struct MotionPipe
{
	HBODY bodies[2];
	HMOTIONNODE mo_nodes[2];
};

typedef HIKLIB_CB(HBODY, FuncBodyInit)(void* paramProc, HCONF conf);
typedef HIKLIB_CB(void, FuncBodyUnInit)(void* paramProc, HBODY body);


HIKLIB(void,			reset_mopipe)(MotionPipe* mopipe);
HIKLIB(bool,			init_mopipe)(MotionPipe* mopipe, const wchar_t* confXML, FuncBodyInit onInitBodyProc[2], void* paramProc);
HIKLIB(bool,			uninit_mopipe)(MotionPipe* mopipe, FuncBodyUnInit onUnInitBodyProc[2], void* paramProc);


HIKLIB(HMOTIONNODE,		create_tree_motion_node)(HBODY mo_src);
HIKLIB(bool,			motion_sync_cnn_homo)(HMOTIONNODE from, HMOTIONNODE to, enum CNN type);
HIKLIB(bool,			motion_sync_cnn_cross_c)(HMOTIONNODE from, HMOTIONNODE to, enum CNN type, const char* pairs[][2], int n_pairs, const Real f2t_w[3][3]);
HIKLIB(bool,			motion_sync_cnn_cross_w)(HMOTIONNODE from, HMOTIONNODE to, enum CNN type, const wchar_t* pairs[][2], int n_pairs, const Real f2t_w[3][3]);
HIKLIB(void,			motion_sync)(HMOTIONNODE root);
HIKLIB(void,			destroy_tree_motion_node)(HMOTIONNODE node);
HIKLIB(void,			destroy_tree_motion)(HMOTIONNODE tree);
HIKLIB(HMOTIONNODE,		get_first_child_mo_node)(HMOTIONNODE node);
HIKLIB(HMOTIONNODE,		get_next_sibling_mo_node)(HMOTIONNODE node);

#ifdef __cplusplus
}
#endif