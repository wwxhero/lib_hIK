#include "pch.h"
#include <filesystem>
#include "motion_pipeline.h"
#include "MoNode.hpp"
#include "handle_helper.hpp"
#include "MotionPipeConf.hpp"
#include "bvh.h"
#include "IKGroupTree.hpp"
#include "ArtiBodyFile.hpp"

namespace fs = std::experimental::filesystem;
using namespace CONF;

struct MotionPipeInternal : public MotionPipe
{
	Eigen::Matrix3r dst2src_w;
	enum Type {FK, IK};
	Type type;
	union
	{
		struct
		{
			HBVH bvh;
			int i_frame;
		};
		struct
		{
			CIKGroupNode* root_ik;
			CBodyLogger* logger;
		};
	};
};


void init_mopipe(MotionPipeInternal* mopipe)
{
	mopipe->bodies[0] = H_INVALID;
	mopipe->bodies[1] = H_INVALID;
	mopipe->mo_nodes[0] = H_INVALID;
	mopipe->mo_nodes[1] = H_INVALID;
	mopipe->n_frames = 0;
	mopipe->bvh = H_INVALID;
	mopipe->i_frame = 0;
	mopipe->root_ik = NULL;
	mopipe->logger = NULL;
}

bool InitBody_Internal(HBODY bodySrc
					, const wchar_t* rootConfDir
					, const CMotionPipeConf& mp_conf
					, int i_body
					, HBODY& hBody
					, unsigned int &frames
					, HBVH& hBVH
					, CIKGroupNode* &root_ikGroup
					, CBodyLogger* &logger)
{
	const CBodyConf* body_confs[] = {&mp_conf.Source, &mp_conf.Destination};
	const CBodyConf* body_conf_i = body_confs[i_body];
	bool initialized = false;
	switch(body_conf_i->type())
	{
		case BODY_TYPE::bvh:
		{
			fs::path fullPath(rootConfDir);
			fs::path relpath(body_conf_i->file_w());
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
			Real identity[3][3] = {
				{1, 0, 0},
				{0, 1, 0},
				{0, 0, 1}
			};
			if (!(clone_body_interests_htr(bodySrc, &body_htr_1, matches, n_match, false, identity)  	// body_htr_1 is an intermediate body, orient bone with src bone information
			 			&& clone_body_htr(body_htr_1, &body_htr_2, mp_conf.m_inv))) 			// body_htr_2 is the result, orient bone with the interest bone information
			 		body_htr_2 = H_INVALID;

			CPairsConf::Data_free(matches, n_match);

			#if 0 // defined _DEBUG
				UE_LOG(LogHIK, Display, TEXT("ArtiBody_SIM"));
				DBG_printOutSkeletalHierachy(body_htr_1);
				UE_LOG(LogHIK, Display, TEXT("ArtiBody_SIM2"));
				DBG_printOutSkeletalHierachy(body_htr_2);
			#endif

			if (VALID_HANDLE(body_htr_1))
			 		destroy_tree_body(body_htr_1);
			hBody = body_htr_2;

			root_ikGroup = CIKGroupTree::Generate(CAST_2PBODY(hBody), *body_conf_i);

			const wchar_t* record = body_conf_i->record_w();
			if (NULL != record)
			{
				fs::path fullPath(rootConfDir);
				fs::path relpath(record);
				fullPath.append(relpath);
				try
				{
					static int s_file_id = 0;
					std::stringstream path;
					path << fullPath.generic_u8string().c_str();
					path << "_" << s_file_id ++;
					logger = new CBodyLogger(CAST_2PBODY(hBody), path.str().c_str());
					logger->LogHeader();
				}
				catch(std::string &exp)
				{
					LOGIKVarErr(LogInfoCharPtr, exp.c_str());
					logger = NULL;
				}
			}

			bool valid_fk_body = VALID_HANDLE(hBody);
			bool valid_ik_group = (NULL != root_ikGroup);
			IKAssert(valid_fk_body);
			IKAssert(valid_ik_group);
			initialized = valid_fk_body && valid_ik_group;
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

		for (int r_i = 0; r_i < 3; r_i ++)
			for (int c_i = 0; c_i < 3; c_i++)
				mopipe->src2dst_w[r_i][c_i] = mp_conf->m[r_i][c_i];
		mopipe->dst2src_w << mp_conf->m_inv[0][0], mp_conf->m_inv[0][1], mp_conf->m_inv[0][2],
							 mp_conf->m_inv[1][0], mp_conf->m_inv[1][1], mp_conf->m_inv[1][2],
							 mp_conf->m_inv[2][0], mp_conf->m_inv[2][1], mp_conf->m_inv[2][2];

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

				const wchar_t** nameTargets = NULL;
				int n_targets = bodies_conf_i->Targets_alloc(nameTargets);

				mopipe->bodies[i_bodyConf] = InitBody_External_i(paramProc
														, bodies_conf_i->file_w()
														, namesOnPair
														, n_pairs
														, scales
														, n_scales
														, nameTargets
														, n_targets);

				CPairsConf::Data_free(namesOnPair, n_pairs);
				CBodyConf::Scale_free(scales, n_scales);
				CBodyConf::Targets_free(nameTargets, n_targets);
			}
			else
			{
				fs::path fullPath(confXML);
				HBVH bvh = H_INVALID;
				CIKGroupNode* root_ik = NULL;
				CBodyLogger* logger = NULL;
				bool initialized = InitBody_Internal(body_ref
													, fullPath.parent_path().c_str()
													, *mp_conf
													, i_bodyConf
													, mopipe->bodies[i_bodyConf]
													, mopipe->n_frames
													, bvh
													, root_ik
													, logger);
				IKAssert(initialized);
				// IKAssert(VALID_HANDLE(bvh) == (NULL == root_ik));
				if (VALID_HANDLE(bvh))
				{
					mopipe->bvh = bvh;
					mopipe->i_frame = -1;
					mopipe->type = MotionPipeInternal::FK;
				}
				else
				{
					IKAssert(root_ik);
					mopipe->root_ik = root_ik;
					mopipe->logger = logger;
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
			if (MotionPipeInternal::IK == mopipe->type)
			{
				std::map<std::wstring, CArtiBodyNode*> nameSrc2bodyDst;
				std::map<std::wstring, std::wstring> nameDst2Src;
				mp_conf->Pair.Map(nameDst2Src, false);
				auto OnBody = [&nameDst2Src, &nameSrc2bodyDst] (CArtiBodyNode* body)
							{
								auto it_nameDst2Src = nameDst2Src.find(body->GetName_w());
								if (nameDst2Src.end() != it_nameDst2Src)
								{
									auto nameSrc = it_nameDst2Src->second;
									nameSrc2bodyDst[nameSrc] = body;
								}
							};

				auto OffBody = [] (CArtiBodyNode* body)
							{

							};

				CArtiBodyTree::TraverseDFS(CAST_2PBODY(mopipe->bodies[c_idxFBX])
										, OnBody
										, OffBody);

				Eigen::Matrix3r src2dst_w;
				const Real(*data_src2dst_w)[3] = mopipe->src2dst_w;
				src2dst_w << data_src2dst_w[0][0], data_src2dst_w[0][1], data_src2dst_w[0][2],
							 data_src2dst_w[1][0], data_src2dst_w[1][1], data_src2dst_w[1][2],
							 data_src2dst_w[2][0], data_src2dst_w[2][1], data_src2dst_w[2][2];

				CIKGroupTree::SetupTargets(mopipe->root_ik
										, nameSrc2bodyDst
										, src2dst_w
										, mopipe->dst2src_w);
			}

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
		{
			delete mopipe;
			*pp_mopipe = NULL;
		}
		CMotionPipeConf::UnLoad(mp_conf);
		return ok;
	}
	else
		return false;
}

void unload_mopipe(MotionPipe* a_mopipe)
{
	if (NULL == a_mopipe)
		return;
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
	}

	mopipe->n_frames = 0;

	switch (mopipe->type)
	{
	case MotionPipeInternal::FK:
		unload_bvh(mopipe->bvh);
		mopipe->bvh = H_INVALID;
		mopipe->i_frame = 0;
		break;
	case MotionPipeInternal::IK:
		delete mopipe->root_ik;
		mopipe->root_ik = NULL;
		delete mopipe->logger;
		mopipe->logger = NULL;
		break;
	}
	LOGIKFlush();

	delete mopipe;
}

void fk_update(MotionPipe* a_mopipe, unsigned int i_frame)
{
	PROFILE_FRAME(i_frame);
	MotionPipeInternal* mopipe = static_cast<MotionPipeInternal*>(a_mopipe);
	if (i_frame != mopipe->i_frame)
	{
		IKAssert(MotionPipeInternal::FK == mopipe->type);
		const int c_idxSim = 0;
		pose_body(mopipe->bvh, mopipe->bodies[c_idxSim], i_frame);
		motion_sync(mopipe->mo_nodes[c_idxSim]);
		mopipe->i_frame = i_frame;
	}
}

bool ik_task_update(HBODY body_t, const _TRANSFORM* l2w)
{
	CArtiBodyNode* body_tar = CAST_2PBODY(body_t);
	Transform_TRS l2w_trs(*l2w);
	return body_tar->UpdateGoal(l2w_trs);
}

void ik_update(MotionPipe* mopipe)
{
	MotionPipeInternal* mopipe_internal = static_cast<MotionPipeInternal*>(mopipe);
	IKAssert(MotionPipeInternal::IK == mopipe_internal->type);
	auto OnGroupNode = [](CIKGroupNode* node_this)
					{
						node_this->IKUpdate();
					};

	auto OffGroupNode = [&](CIKGroupNode* node_this)
					{
					};

	CIKGroupTree::TraverseDFS(mopipe_internal->root_ik, OnGroupNode, OffGroupNode);
	const int c_idxSim = 0;
	motion_sync(mopipe->mo_nodes[c_idxSim]);
	if (NULL != mopipe_internal->logger)
		mopipe_internal->logger->LogMotion();
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
