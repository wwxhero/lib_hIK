#include "pch.h"
#include <opencv2/opencv.hpp>
#include <fstream>
#include "Math.hpp"
#include "posture_graph.h"
#include "ArtiBody.hpp"
#include "ArtiBodyFile.hpp"
#include "MotionPipeConf.hpp"
#include "IKGroupTree.hpp"
#include "filesystem_helper.hpp"
#include "PostureGraph.hpp"

namespace CONF
{
	class CInterestsConf : public ConfDoc<CInterestsConf>
	{
	public:
		CInterestsConf()
		{

		}
		bool Initialize(const TiXmlNode* doc)
		{
			auto OnTraverXmlNode = [this](const TiXmlNode* node) -> bool
			{
				bool ret = true;
				bool is_a_source = false;
				bool is_a_desti = false;
				if (TiXmlNode::ELEMENT == node->Type())
				{
					auto name = node->ValueStr();
					const TiXmlElement* ele = node->ToElement();
					if ("Joint" == name)
					{
						const char* name = ele->Attribute("name");
						Joints.push_back(std::string(name));
					}

				}
				return ret;
			};

			return (TraverseBFS_XML_tree(doc, OnTraverXmlNode));
		}

		void Dump() const
		{
			std::cout << "<Interests>" << std::endl;
			for (auto name : Joints)
				std::cout << "\t<Joint name=\"" << name << "\"/>" << std::endl;
			std::cout << "</Interests>" << std::endl;
		}
	public:
		std::list<std::string> Joints;
	};
};

bool err_vis(const char* interests_conf_path, const char* path_htr, const char* path_png)
{
	try
	{
		CONF::CInterestsConf* interests_conf = CONF::CInterestsConf::Load(interests_conf_path);
		if (!interests_conf_path)
		{
			std::stringstream err;
			err << "loading " << interests_conf_path << " failed";
			LOGIKVarErr(LogInfoCharPtr, err.str().c_str());
			return false;
		}
		/*else
		{
			interests_conf->Dump();
		}*/

		CFile2ArtiBody htr2body(path_htr);
		Eigen::MatrixXr err_out;
		htr2body.ETB_Setup(err_out, interests_conf->Joints);
		CONF::CInterestsConf::UnLoad(interests_conf);
		int n_rows = (int)err_out.rows();
		int n_cols = (int)err_out.cols();
		IKAssert(n_rows == n_cols);
		cv::Mat err_png(n_rows, n_cols, CV_16U);
		for (int i_row = 0; i_row < n_rows; i_row++)
			for (int i_col = 0; i_col < n_cols; i_col++)
				err_png.at<unsigned short>(i_row, i_col) = (unsigned short)(err_out(i_row, i_col)*(Real)USHRT_MAX);
		imwrite(path_png, err_png);
	}
	catch(const std::string& err)
	{
		LOGIKVarErr(LogInfoCharPtr, err.c_str());
		return false;
	}
	return true;
}




bool dissect(const char* confXML, const char* path_htr, const char* dir_out)
{
	CONF::CBodyConf* body_conf = NULL;
	CArtiBodyNode* body_root = NULL;
	CIKGroupNode* ik_group = NULL;
	bool ok = true;
	struct Bound
	{
		CArtiBodyNode* group_root;
		CArtiBody2File* group_file;
	};
	std::vector<Bound> section;
	try
	{
		std::string err;
		body_conf = CONF::CBodyConf::Load(confXML);
		if (NULL == body_conf)
		{
			err = "body conf xml does not exist!!!";
			LOGIKVarErr(LogInfoCharPtr, err.c_str());
			ok = false;
			goto EXIT;
		}

		CFile2ArtiBody htr(path_htr);
		body_root = htr.CreateBody(BODY_TYPE::htr);
		IKAssert(NULL != body_root);
		ik_group = CIKGroupTree::Generate(body_root, *body_conf);
		if (NULL == ik_group)
		{
			err = "IK group is not created, confirm with body conf xml file!!!";
			LOGIKVarErr(LogInfoCharPtr, err.c_str());
			ok = false;
			goto EXIT;
		}

		int n_frames = htr.frames();
		auto OnGroupNode = [&section, n_frames](CIKGroupNode* g_node)
			{
				if (!g_node->Empty())
				{
					CArtiBodyNode* group_root = g_node->RootBody();
					CArtiBody2File* group_file = new CArtiBody2File(group_root, n_frames);
					section.push_back({group_root, group_file});
				}
			};

		auto OffGroupNode = [](CIKGroupNode* g_node)
			{
			};

		CIKGroupTree::TraverseDFS(ik_group, OnGroupNode, OffGroupNode);

		for (int i_frame = 0; i_frame < n_frames; i_frame++)
		{
			htr.UpdateMotion(i_frame, body_root);
			for (auto sec : section)
			{
				CArtiBodyTree::FK_Update<true>(sec.group_root);
				sec.group_file->UpdateMotion(i_frame);
			}
		}

		fs::path out_path_dir(dir_out);
		for (auto sec : section)
		{
			fs::path out_path(out_path_dir);
			std::string file_htr(sec.group_root->GetName_c()); file_htr += ".htr";
			out_path.append(file_htr);
			sec.group_file->WriteBvhFile(out_path.u8string().c_str());

		}
	}
	catch(const std::string& exp)
	{
		LOGIKVarErr(LogInfoCharPtr, exp.c_str());
		ok = false;
	}

EXIT:

	for (auto sec : section)
		delete sec.group_file;

	if (NULL != ik_group)
		CIKGroupTree::Destroy(ik_group);
	if (NULL != body_root)
		CArtiBodyTree::Destroy(body_root);
	if (NULL != body_conf)
		CONF::CBodyConf::UnLoad(body_conf);
	return ok;
}


bool posture_graph_gen(const char* interests_conf_path, const char* path_htr, const char* dir_out)
{
	bool ok = false;
	CPostureGraphGen* pg_gen = NULL;
	try
	{
		CFile2ArtiBody htr2body(path_htr);
		fs::path path_err_tb(path_htr);
		path_err_tb.replace_extension(".png");
		unsigned int n_frames = htr2body.frames();
		cv::Mat err_tb = cv::imread(path_err_tb.u8string(), cv::IMREAD_GRAYSCALE);
		if (NULL == err_tb.data)
		{
			CONF::CInterestsConf* interests_conf = CONF::CInterestsConf::Load(interests_conf_path);
			if (!interests_conf_path)
			{
				std::stringstream err;
				err << "loading " << interests_conf_path << " failed";
				LOGIKVarErr(LogInfoCharPtr, err.str().c_str());
				return false;
			}
			// htr2body.ETB_Setup(err_tb, interests_conf->Joints);
			CONF::CInterestsConf::UnLoad(interests_conf);
		}
		// pg_gen = CPostureGraphGen::Generate(&htr2body, );
		pg_gen->Save(dir_out);
		ok = true;
	}
	catch (std::string& err)
	{
		LOGIKVarErr(LogInfoCharPtr, err.c_str());
		ok = false;
	}

	CPostureGraphGen::Destroy(pg_gen);
	return ok;
}