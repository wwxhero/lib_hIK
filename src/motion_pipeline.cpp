#include "pch.h"
#include "motion_pipeline.h"
#include "MoNode.h"
#include "handle_helper.hpp"
#include "MotionPipeConf.hpp"

void init_mopipe(MotionPipe* mopipe)
{
	mopipe->bodies[0] = H_INVALID;
	mopipe->bodies[1] = H_INVALID;
	mopipe->mo_nodes[0] = H_INVALID;
	mopipe->mo_nodes[1] = H_INVALID;
}

HBODY InitBody_Internal(HBODY bodySrc, const CMotionPipeConf& mp_conf, int i_body, unsigned int &frames)
{
	//fixme: to initiailize a body
	return H_INVALID;
}

bool load_mopipe(MotionPipe* mopipe, const wchar_t* confXML, FuncBodyInit onInitBodyProc[2], void* paramProc)
{
	LOGIKVar(LogInfoWCharPtr, confXML);
	init_mopipe(mopipe);
	CMotionPipeConf* mp_conf = CMotionPipeConf::Load(confXML);
	if (NULL != mp_conf)
	{
#ifdef _DEBUG
		mp_conf->Dump_Dbg();
#endif
		const int c_idxFBX = 1;
		const int c_idxSim = 0;

		const CBodyConf* bodies_conf[] = { &mp_conf->Source, &mp_conf->Destination };

		HBODY body_ref = H_INVALID;

		// initialize bodies
		for (int i_bodyConf = c_idxFBX; i_bodyConf > -1; i_bodyConf --)
		{
			auto InitBody_External_i = onInitBodyProc[i_bodyConf];
			if (NULL != InitBody_External_i)
			{
				auto* bodies_conf_i = bodies_conf[i_bodyConf];

				const wchar_t** namesOnPair = NULL;
				int n_pairs = mp_conf->Pair.Data(i_bodyConf, namesOnPair);

				B_Scale* scales = NULL;
				int n_scales = bodies_conf_i->Scale(scales);

				const wchar_t** namesEEFs = NULL;
				int n_eefs = bodies_conf_i->EndEEF(namesEEFs);

				mopipe->bodies[i_bodyConf] = InitBody_External_i(paramProc
														, bodies_conf_i->file()
														, namesOnPair
														, n_pairs
														, scales
														, n_scales
														, namesEEFs
														, n_eefs);
			}
			else
			{
				mopipe->bodies[i_bodyConf] = InitBody_Internal(body_ref, *mp_conf, i_bodyConf, mopipe->n_frames);
			}
			body_ref = mopipe->bodies[i_bodyConf];
		}

		// initialize motion pipe
		HBODY body_fbx = mopipe->bodies[c_idxFBX];
		HBODY body_sim = mopipe->bodies[c_idxSim];

		bool fbx_created = VALID_HANDLE(body_fbx);
		LOGIKVar(LogInfoBool, fbx_created);
		bool sim_created = VALID_HANDLE(body_sim);
		LOGIKVar(LogInfoBool, sim_created);

		bool ok = fbx_created && sim_created;

		if (ok)
		{
			for (int r_i = 0; r_i < 3; r_i ++)
				for (int c_i = 0; c_i < 3; c_i++)
					mopipe->src2dst_w[r_i][c_i] = mp_conf->m[r_i][c_i];

		 	auto moDriver = create_tree_motion_node(mopipe->bodies[0]);
		 	auto moDrivee = create_tree_motion_node(mopipe->bodies[1]);
		 	bool mo_bvh_created = VALID_HANDLE(moDriver);
		 	bool mo_drv_created = VALID_HANDLE(moDrivee);
		 	bool sync_cross = (mp_conf->sync == CMotionPipeConf::cross);
		 	bool cnn_bvh2htr = false;

		 	if (mo_bvh_created && mo_drv_created)
		 	{
		 		if (sync_cross)
		 		{
		 			const wchar_t* (*matches)[2] = NULL;
		 			int n_match = mp_conf->Pair.Data(&matches);
		 			cnn_bvh2htr = motion_sync_cnn_cross_w(moDriver, moDrivee, FIRSTCHD, matches, n_match, mopipe->src2dst_w);
		 		}
		 		else
		 			cnn_bvh2htr = motion_sync_cnn_homo(moDrivee, moDrivee, FIRSTCHD);
		 	}

		 	ok =  (mo_bvh_created
		 		&& mo_drv_created
		 		&& cnn_bvh2htr);
			LOGIKVar(LogInfoBool, mo_bvh_created);
			LOGIKVar(LogInfoBool, mo_drv_created);
			LOGIKVar(LogInfoBool, cnn_bvh2htr);

			mopipe->mo_nodes[0] = moDriver;
			mopipe->mo_nodes[1] = moDrivee;
		}
		CMotionPipeConf::UnLoad(mp_conf);
		// return ok;
		return false;
	}
	else
		return false;
}

void unload_mopipe(MotionPipe* mopipe)
{
	for (int i_retar = 0; i_retar < 2; i_retar ++)
	{
		auto& moNode_i = mopipe->mo_nodes[i_retar];
		if (VALID_HANDLE(moNode_i))
			destroy_tree_motion_node(moNode_i);
		moNode_i = H_INVALID;

		auto& body_i = mopipe->bodies[i_retar];
		if (VALID_HANDLE(body_i))
			destroy_tree_body(body_i);
		body_i = H_INVALID;
	}
}

HMOTIONNODE	create_tree_motion_node(HBODY mo_src)
{
	//todo: create a tree node that represents motions of the articulated body mo_src
	CArtiBodyNode* body = CAST_2PBODY(mo_src);
	CMoNode* mo = new CMoNode(body);
	return CAST_2HMONODE(mo);
}

void destroy_tree_motion_node(HMOTIONNODE node)
{
	//todo: destroy motion node
	CMoNode* mo = CAST_2PMONODE(node);
	delete mo;
}

void destroy_tree_motion(HMOTIONNODE tree)
{
	CMoNode* mo = CAST_2PMONODE(tree);
	CMoTree::Destroy(mo);
}

bool motion_sync_cnn_homo(HMOTIONNODE from, HMOTIONNODE to, CNN type)
{
	//todo: connection to motion nodes as a homo-space map
	CMoNode* mo_from = CAST_2PMONODE(from);
	CMoNode* mo_to = CAST_2PMONODE(to);
	return CMoTree::Connect_homo(mo_from, mo_to, type);
}

bool motion_sync_cnn_cross_c(HMOTIONNODE from
							, HMOTIONNODE to
							, CNN type
							, const char* pairs[][2]
							, int n_pairs
							, const Real p2c_w[3][3])
{
	//todo: connection to motion nodes as a cross-space map
	CMoNode* mo_from = CAST_2PMONODE(from);
	CMoNode* mo_to = CAST_2PMONODE(to);
	return CMoTree::Connect_cross(mo_from, mo_to, type, pairs, n_pairs, p2c_w);
}

bool motion_sync_cnn_cross_w(HMOTIONNODE from
							, HMOTIONNODE to
							, CNN type
							, const wchar_t* pairs[][2]
							, int n_pairs
							, const Real p2c_w[3][3])
{
	//todo: connection to motion nodes as a cross-space map
	CMoNode* mo_from = CAST_2PMONODE(from);
	CMoNode* mo_to = CAST_2PMONODE(to);
	return CMoTree::Connect_cross(mo_from, mo_to, type, pairs, n_pairs, p2c_w);
}

void motion_sync(HMOTIONNODE root)
{
	//todo: sync motions from tree root to leaf
	CMoNode* mo_root = CAST_2PMONODE(root);
	CMoTree::Motion_sync(mo_root);
}

HMOTIONNODE	get_first_child_mo_node(HMOTIONNODE node)
{
	CMoNode* mo_node = CAST_2PMONODE(node);
	return CAST_2HMONODE(mo_node->GetFirstChild());
}

HMOTIONNODE	get_next_sibling_mo_node(HMOTIONNODE node)
{
	CMoNode* mo_node = CAST_2PMONODE(node);
	return CAST_2HMONODE(mo_node->GetNextSibling());
}
