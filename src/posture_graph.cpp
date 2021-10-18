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