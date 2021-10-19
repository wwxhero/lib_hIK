#include "pch.h"
#include <opencv2/opencv.hpp>
#include "Math.hpp"
#include "posture_graph.h"
#include "ArtiBody.hpp"
#include "ArtiBodyFile.hpp"

void err_vis(const char* path_htr, const char* path_png)
{
	try
	{
		CFile2ArtiBody htr2body(path_htr);
		unsigned int n_frames = htr2body.frames();
		cv::Mat err_out(n_frames, n_frames, CV_16U);
		CArtiBodyNode* body_i = htr2body.CreateBody(BODY_TYPE::htr);
		TransformArchive tm_data_i;
		CArtiBodyNode* body_j = htr2body.CreateBody(BODY_TYPE::htr);
		TransformArchive tm_data_j;
		for (unsigned int i_frame = 0; i_frame < n_frames; i_frame++)
		{
			htr2body.UpdateMotion(i_frame, body_i);
			CArtiBodyTree::Serialize<true>(body_i, tm_data_i);
			for (unsigned int j_frame = 0; j_frame < n_frames; j_frame++)
			{
				htr2body.UpdateMotion(j_frame, body_j);
				CArtiBodyTree::Serialize<true>(body_j, tm_data_j);
				auto& vis_scale_ij = err_out.at<unsigned short>(i_frame, j_frame);
				auto err_ij = TransformArchive::Error_q(tm_data_i, tm_data_j);
				vis_scale_ij = (unsigned short)(err_ij * USHRT_MAX);
			}
		}
		CArtiBodyTree::Destroy(body_i);
		CArtiBodyTree::Destroy(body_j);
		imwrite(path_png, err_out);
	}
	catch(const std::string& err)
	{
		LOGIKVarErr(LogInfoCharPtr, err.c_str());
	}
}


//bool InitBody_Internal_ik(const CMotionPipeConf& mp_conf
//						, const CBodyConf* body_conf_i
//						, HBODY& hBody
//						, CIKGroupNode* &root_ikGroup)
//{
//	bool initialized = false;
//
//	IKAssert(VALID_HANDLE(bodySrc));
//	const wchar_t* (*matches)[2] = NULL;
//	int n_match = mp_conf.Pair.Data_alloc(&matches);
//	HBODY body_htr_1 = H_INVALID;
//	HBODY body_htr_2 = H_INVALID;
//	Real identity[3][3] = {
//		{1, 0, 0},
//		{0, 1, 0},
//		{0, 0, 1}
//	};
//	if (!(clone_body_interests_htr(bodySrc, &body_htr_1, matches, n_match, false, identity)  	// body_htr_1 is an intermediate body, orient bone with src bone information
//	 			&& clone_body_htr(body_htr_1, &body_htr_2, mp_conf.m_inv))) 			// body_htr_2 is the result, orient bone with the interest bone information
//	 		body_htr_2 = H_INVALID;
//
//	CPairsConf::Data_free(matches, n_match);
//
//	#if 0 // defined _DEBUG
//		UE_LOG(LogHIK, Display, TEXT("ArtiBody_SIM"));
//		DBG_printOutSkeletalHierachy(body_htr_1);
//		UE_LOG(LogHIK, Display, TEXT("ArtiBody_SIM2"));
//		DBG_printOutSkeletalHierachy(body_htr_2);
//	#endif
//
//	if (VALID_HANDLE(body_htr_1))
//	 		destroy_tree_body(body_htr_1);
//	hBody = body_htr_2;
//
//	root_ikGroup = CIKGroupTree::Generate(CAST_2PBODY(hBody), *body_conf_i);
//
//
//
//	bool valid_fk_body = VALID_HANDLE(hBody);
//	bool valid_ik_group = (NULL != root_ikGroup);
//	IKAssert(valid_fk_body);
//	IKAssert(valid_ik_group);
//	initialized = valid_fk_body && valid_ik_group;
//	return initialized;
//}

void dissect(const char* confXML, const char* path_htr, const char* dir_out)
{

}