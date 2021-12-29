#pragma once
#include "articulated_body.h"
#ifdef __cplusplus
extern "C" {
#endif

struct MotionPipe
{
	HBODY bodies[2];
	HMOTIONNODE mo_nodes[2];
	Real src2dst_w[3][3];
	unsigned int n_frames; // number of frames for FK or 0 for HIK
};

typedef struct _B_Scale
{
	const wchar_t* bone_name;
	Real scaleX;
	Real scaleY;
	Real scaleZ;
} B_Scale;

typedef HIKLIB_CB(HBODY, *FuncBodyInit)(void* paramProc
									, const wchar_t* filePath
									, const wchar_t* namesOnPair[]
									, int n_pairs
									, const B_Scale scales[]
									, int n_scales
									, const wchar_t* nameTargets[]
									, int n_targets);

HIKLIB(bool,			load_mopipe)(MotionPipe** pp_mopipe, const wchar_t* confXML, FuncBodyInit onInitBodyProc[2], void* paramProc);
HIKLIB(void,			unload_mopipe)(MotionPipe* mopipe);
HIKLIB(void,			fk_update)(MotionPipe* mopipe, unsigned int i_frame);
HIKLIB(bool,			ik_task_update)(HBODY body_t, const _TRANSFORM* tm);
HIKLIB(void,			ik_update)(MotionPipe* mopipe);
HIKLIB(void,			ik_reset)(MotionPipe* mopipe);

// these APIs are not for game engine usage
HIKLIB(HMOTIONNODE,		create_tree_motion_node)(HBODY mo_src);
HIKLIB(bool,			motion_sync_cnn_homo)(HMOTIONNODE from, HMOTIONNODE to, enum CNN type);
HIKLIB(bool,			motion_sync_cnn_cross_c)(HMOTIONNODE from, HMOTIONNODE to, enum CNN type, const char* pairs[][2], int n_pairs, const Real f2t_w[3][3]);
HIKLIB(bool,			motion_sync_cnn_cross_w)(HMOTIONNODE from, HMOTIONNODE to, enum CNN type, const wchar_t* pairs[][2], int n_pairs, const Real f2t_w[3][3]);
HIKLIB(void,			destroy_tree_motion_node)(HMOTIONNODE node);
HIKLIB(void,			destroy_tree_motion)(HMOTIONNODE tree);
HIKLIB(void,			motion_sync)(HMOTIONNODE root);
HIKLIB(HMOTIONNODE,		get_first_child_mo_node)(HMOTIONNODE node);
HIKLIB(HMOTIONNODE,		get_next_sibling_mo_node)(HMOTIONNODE node);

#ifdef __cplusplus
}
#endif