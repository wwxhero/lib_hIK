#include "pch.h"
#include <filesystem>
#include "motion_pipeline.h"
#include "MoNode.h"
#include "handle_helper.hpp"
#include "MotionPipeConf.hpp"
#include "bvh.h"

using namespace CONF;

class CIKGroupNode
{

};

struct MotionPipeInternal : public MotionPipe
{
	enum Type {FK, IK};
	Type type;
	union
	{
		HBVH bvh;		//fixme: replace this handle with the pointer
		CIKGroupNode* root_ik;
	};
};


void init_mopipe(MotionPipeInternal* mopipe)
{
	mopipe->bodies[0] = H_INVALID;
	mopipe->bodies[1] = H_INVALID;
	mopipe->mo_nodes[0] = H_INVALID;
	mopipe->mo_nodes[1] = H_INVALID;
	mopipe->n_frames = 0;
	mopipe->root_ik = NULL;
}

bool InitBody_Internal(HBODY bodySrc
					, const wchar_t* rootConfDir
					, const CMotionPipeConf& mp_conf
					, int i_body
					, HBODY& hBody
					, unsigned int &frames
					, HBVH& hBVH
					, CIKGroupNode* &root_ikGroup)
{
	const CBodyConf* body_confs[] = {&mp_conf.Source, &mp_conf.Destination};
	const CBodyConf* body_conf_i = body_confs[i_body];
	bool initialized = false;
	switch(body_conf_i->type())
	{
		case BODY_TYPE::bvh:
		{
			std::experimental::filesystem::path fullPath(rootConfDir);
			std::experimental::filesystem::path relpath(body_conf_i->file_w());
			fullPath.append(relpath);
			hBVH = load_bvh_w(fullPath.c_str());
			IKAssert(VALID_HANDLE(hBVH));
			bool bvh_load = (VALID_HANDLE(hBVH)
			 			&& VALID_HANDLE(hBody = create_tree_body_bvh(hBVH)));
		 	LOGIKVar(LogInfoBool, bvh_load);
		 	if (bvh_load)
		 		frames = get_n_frames(hBVH);
		 	initialized = bvh_load;
		 	break;
		}

		case BODY_TYPE::htr:
		{
			IKAssert(VALID_HANDLE(bodySrc));
			const wchar_t* (*matches)[2] = NULL;
			int n_match = mp_conf.Pair.Data_alloc(&matches);
			HBODY body_htr_1 = H_INVALID;
			HBODY body_htr_2 = H_INVALID;
			if (!(clone_body_interests(bodySrc, &body_htr_1, matches, n_match, false)  	// body_htr_1 is an intermediate body, orient bone with src bone information
			 			&& clone_body(body_htr_1, htr, &body_htr_2))) 						    // body_htr_2 is the result, orient bone with the interest bone information
			 		body_htr_2 = H_INVALID;

			CPairsConf::Data_free(matches, n_match);

			#if 0 // defined _DEBUG
				UE_LOG(LogHIK, Display, TEXT("ArtiBody_SIM"));
				DBG_printOutSkeletalHierachy(body_htr_1);
				UE_LOG(LogHIK, Display, TEXT("ArtiBody_SIM2"));
				DBG_printOutSkeletalHierachy(body_htr_2);
			#endif

			initialized = VALID_HANDLE(body_htr_1)
						&& VALID_HANDLE(body_htr_2);
			if (VALID_HANDLE(body_htr_1))
			 		destroy_tree_body(body_htr_1);
			hBody = body_htr_2;
			break;
		}

		case BODY_TYPE::fbx:
		{
			assert(0); // fbx parsing is not supported by HIK
			break;
		}

	}

	return initialized;
}

bool load_mopipe(MotionPipe** pp_mopipe, const wchar_t* confXML, FuncBodyInit onInitBodyProc[2], void* paramProc)
{
	LOGIKVar(LogInfoWCharPtr, confXML);
	CMotionPipeConf* mp_conf = CMotionPipeConf::Load(confXML);

	if (NULL != mp_conf)
	{
#ifdef _DEBUG
		mp_conf->Dump_Dbg();
#endif
		MotionPipeInternal* mopipe = new MotionPipeInternal;
		init_mopipe(mopipe);

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
				int n_pairs = mp_conf->Pair.Data_alloc(i_bodyConf, namesOnPair);

				B_Scale* scales = NULL;
				int n_scales = bodies_conf_i->Scale_alloc(scales);

				const wchar_t** namesEEFs = NULL;
				int n_eefs = bodies_conf_i->EndEEF_alloc(namesEEFs);

				mopipe->bodies[i_bodyConf] = InitBody_External_i(paramProc
														, bodies_conf_i->file_w()
														, namesOnPair
														, n_pairs
														, scales
														, n_scales
														, namesEEFs
														, n_eefs);

				CPairsConf::Data_free(namesOnPair, n_pairs);
				CBodyConf::Scale_free(scales, n_scales);
				CBodyConf::EndEEF_free(namesEEFs, n_eefs);
			}
			else
			{
				std::experimental::filesystem::path fullPath(confXML);
				HBVH bvh = H_INVALID;
				CIKGroupNode* root_ik = NULL;
				InitBody_Internal(body_ref
								, fullPath.parent_path().c_str()
								, *mp_conf
								, i_bodyConf
								, mopipe->bodies[i_bodyConf]
								, mopipe->n_frames
								, bvh
								, root_ik);
				// IKAssert(VALID_HANDLE(bvh) == (NULL == root_ik));
				if (VALID_HANDLE(bvh))
				{
					mopipe->bvh = bvh;
					mopipe->type = MotionPipeInternal::FK;
				}
				else
				{
					mopipe->root_ik = root_ik;
					mopipe->type = MotionPipeInternal::IK;
				}
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
		 	bool sync_cross = (mp_conf->sync == CMoNode::cross);
		 	bool cnn_bvh2htr = false;

		 	if (mo_bvh_created && mo_drv_created)
		 	{
		 		if (sync_cross)
		 		{
		 			const wchar_t* (*matches)[2] = NULL;
		 			int n_match = mp_conf->Pair.Data_alloc(&matches);
		 			cnn_bvh2htr = motion_sync_cnn_cross_w(moDriver, moDrivee, FIRSTCHD, matches, n_match, mopipe->src2dst_w);
		 			CPairsConf::Data_free(matches, n_match);
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
			*pp_mopipe = mopipe;
		}
		else
			delete mopipe;
		CMotionPipeConf::UnLoad(mp_conf);
		return ok;
	}
	else
		return false;
}

void unload_mopipe(MotionPipe* a_mopipe)
{
	MotionPipeInternal* mopipe = static_cast<MotionPipeInternal*>(a_mopipe);
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

		mopipe->n_frames = 0;
		switch (mopipe->type)
		{
		case MotionPipeInternal::FK:
			unload_bvh(mopipe->bvh);
			mopipe->bvh = H_INVALID;
			break;
		case MotionPipeInternal::IK:
			break;
		}
	}
	delete mopipe;
}

void motion_sync_to(MotionPipe* a_mopipe, unsigned int i_frame)
{
	MotionPipeInternal* mopipe = static_cast<MotionPipeInternal*>(a_mopipe);
	IKAssert(mopipe->type == MotionPipeInternal::FK);
	const int c_idxSim = 0;
	pose_body(mopipe->bvh, mopipe->bodies[c_idxSim], i_frame);
	motion_sync(mopipe->mo_nodes[c_idxSim]);
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
