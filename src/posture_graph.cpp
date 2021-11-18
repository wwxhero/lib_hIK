#include "pch.h"
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

bool init_err_tb(const char* interests_conf_path, const char* path_htr, _ERROR_TB* err_tb)
{
	try
	{
		err_tb->data = NULL;
		err_tb->n_rows = 0;
		err_tb->n_cols = 0;
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
		err_tb->n_rows = (int)err_out.rows();
		err_tb->n_cols = (int)err_out.cols();
		auto data_size = err_out.rows() * err_out.cols() * sizeof(Real);
		err_tb->data = (Real*)malloc(data_size);
		memcpy(err_tb->data, err_out.data(), data_size); //err_out is a column major matrix
	}
	catch(const std::string& err)
	{
		LOGIKVarErr(LogInfoCharPtr, err.c_str());
		return false;
	}
	return true;
}

bool init_err_tb_merged(const char* interests_conf_path, const char* pg_theta_0, const char* pg_theta_1, _ERROR_TB* err_tb)
{
	try
	{
		err_tb->data = NULL;
		err_tb->n_rows = 0;
		err_tb->n_cols = 0;
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

		CFile2ArtiBody htr2body_0(pg_theta_0);
		CFile2ArtiBody htr2body_1(pg_theta_1);

		bool merged = htr2body_0.Merge(htr2body_1);
		if (merged)
		{
			Eigen::MatrixXr err_out;
			htr2body_0.ETB_Setup(err_out, interests_conf->Joints);
			err_tb->n_rows = (int)err_out.rows();
			err_tb->n_cols = (int)err_out.cols();
			auto data_size = err_out.rows() * err_out.cols() * sizeof(Real);
			err_tb->data = (Real*)malloc(data_size);
			memcpy(err_tb->data, err_out.data(), data_size); //err_out is a column major matrix
		}
		return merged;
	}
	catch(const std::string& err)
	{
		LOGIKVarErr(LogInfoCharPtr, err.c_str());
		return false;
	}
	return true;
}

void uninit_err_tb(_ERROR_TB* err_tb)
{
	free(err_tb->data);
	err_tb->data = NULL;
	err_tb->n_rows = 0;
	err_tb->n_cols = 0;
}

Real err_entry(const _ERROR_TB* err_tb, int i_row, int i_col)
{
	IKAssert(i_row < err_tb->n_rows && i_col < err_tb->n_cols);
	return err_tb->data[i_col*err_tb->n_rows + i_row];
}

bool dissect(const char* confXML, const char* path, const char* dir_out)
{
	CONF::CBodyConf* body_conf = NULL;
	CIKGroupNode* ik_group = NULL;
	bool ok = true;
	struct Bound
	{
		CArtiBodyNode* group_root;
		CArtiBodyRef2File* group_file;
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

		CFile2ArtiBody file(path);
		const CArtiBodyNode* body_root = file.GetBody();
		IKAssert(NULL != body_root);
		ik_group = CIKGroupTree::Generate(body_root, *body_conf);
		if (NULL == ik_group)
		{
			err = "IK group is not created, confirm with body conf xml file!!!";
			LOGIKVarErr(LogInfoCharPtr, err.c_str());
			ok = false;
			goto EXIT;
		}

		int n_frames = file.frames();
		auto OnGroupNode = [&section, n_frames](CIKGroupNode* g_node)
			{
				if (!g_node->Empty())
				{
					CArtiBodyNode* group_root = g_node->RootBody();
					CArtiBodyRef2File* group_file = new CArtiBodyRef2File(group_root, n_frames);
					section.push_back({group_root, group_file});
				}
			};

		auto OffGroupNode = [](CIKGroupNode* g_node)
			{
			};

		CIKGroupTree::TraverseDFS(ik_group, OnGroupNode, OffGroupNode);

		for (int i_frame = 0; i_frame < n_frames; i_frame++)
		{
			file.PoseBody<false>(i_frame);
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
			std::string file_name(sec.group_root->GetName_c());
			if (htr == file.GetBody()->c_type)
				file_name += ".htr";
			else
				file_name += ".bvh";
			out_path.append(file_name);
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
	if (NULL != body_conf)
		CONF::CBodyConf::UnLoad(body_conf);
	return ok;
}


bool posture_graph_gen(const char* interests_conf_path, const char* path_htr, const char* dir_out, Real epsErr, const _ERROR_TB* err_tb_exter)
{
	bool ok = false;
	try
	{
		CFile2ArtiBody htr2body(path_htr);

		CONF::CInterestsConf* interests_conf = CONF::CInterestsConf::Load(interests_conf_path);
		if (NULL == interests_conf)
		{
			std::stringstream err;
			err << "loading " << interests_conf_path << " failed";
			LOGIKVarErr(LogInfoCharPtr, err.str().c_str());
			return false;
		}

		Eigen::MatrixXr err_tb;

		if (NULL == err_tb_exter)
			htr2body.ETB_Setup(err_tb, interests_conf->Joints);
		else
		{
			err_tb = Eigen::Map<Eigen::MatrixXr>(err_tb_exter->data
												, err_tb_exter->n_rows
												, err_tb_exter->n_cols);
		}

		CONF::CInterestsConf::UnLoad(interests_conf);

		CPostureGraphOpen pg_epsilon(&htr2body);
		const int T_PID = 0;
		std::vector<int> postures_T = {T_PID};
		CPostureGraphOpen::InitTransitions(pg_epsilon, err_tb, epsErr, postures_T);
		CPostureGraphClose2File* pg_gen = CPostureGraphOpen::GenerateClosePG(pg_epsilon, err_tb, T_PID);
		ok = (NULL != pg_gen);
		if (ok)
		{
			pg_gen->Save(dir_out);
			delete pg_gen;
		}
	}
	catch (std::string& err)
	{
		LOGIKVarErr(LogInfoCharPtr, err.c_str());
		ok = false;
	}
	return ok;
}

bool posture_graph_merge(const char* interests_conf_path, const char* pg_dir_0, const char* pg_dir_1, const char* pg_name, Real eps_err)
{
	// bool ok = false;
	// try
	// {
	// 	CFile2PostureGraphClose pg_0, pg_1;
	// 	pg_0.Load(pg_dir_0, pg_name);
	// 	pg_1.Load(pg_dir_1, pg_name);
	// 	CFile2ArtiBody theta_0(pg_0.Theta());
	// 	CFile2ArtiBody theta_1(pg_1.Theta());
	// 	CFile2ArtiBody& theta = theta_0;
	// 	ok = theta.Merge(theta_1);
	// 	if (ok)
	// 	{
	// 		CONF::CInterestsConf* interests_conf = CONF::CInterestsConf::Load(interests_conf_path);
	// 		if (NULL == interests_conf)
	// 		{
	// 			std::stringstream err;
	// 			err << "loading " << interests_conf_path << " failed";
	// 			LOGIKVarErr(LogInfoCharPtr, err.str().c_str());
	// 			return false;
	// 		}

	// 		Eigen::MatrixXr err_tb;
	// 		htr2body.ETB_Setup(err_tb, interests_conf->Joints);

	// 		CONF::CInterestsConf::UnLoad(interests_conf);

	// 		CPostureGraphOpen pg_open(theta);
	// 		const int T_PID0 = 0;
	// 		const int T_PID1 = pg_0.N_theta();
	// 		std::vector<int> postures_T = {T_PID0, T_PID1};
	// 		CPostureGraphOpen::MergeTransitions(pg_open, pg_0, pg_1, err_tb, epsErr, postures_T);
	// 		CPostureGraphClose2File* pg_gen = CPostureGraphOpen::GenerateClosePG(pg_open, err_tb, T_PID0);
	// 		ok = (NULL != pg_gen);
	// 		if (ok)
	// 		{
	// 			pg_gen->Save(dir_out);
	// 			delete pg_gen;
	// 		}
	// 	}
	// }
	// catch (std::string& err)
	// {
	// 	LOGIKVarErr(LogInfoCharPtr, err.c_str());
	// 	ok = false;
	// }
	// return ok;
	return false;
}

bool convert_pg2dot(const char* path_src, const char* path_dst)
{
	CFile2PostureGraphClose filePG;
	filePG.LoadTransitions(path_src);
	filePG.SaveTransitions(path_dst, F_DOT);
	return true;
}

bool trim(const char* src, const char* dst, const char* const names_rm[], int n_names)
{
	bool ret = false;
	try
	{
		CFile2ArtiBody file2body(src);
		CArtiBodyNode* rootTrim = NULL;
		ret = CArtiBodyTree::Clone(file2body.GetBody(), &rootTrim);
		if (ret)
		{
			std::set<std::string> rms;
			for (int i_name = 0; i_name < n_names; i_name ++)
				rms.insert(names_rm[i_name]);

			auto OnEnterBodyTrim = [](CArtiBodyNode* node)
				{
				};

			auto OnLeaveBodyTrim = [&rms = std::as_const(rms)](CArtiBodyNode* node)
				{
					bool rm_node = (rms.end() != rms.find(node->GetName_c()));
					if (rm_node)
					{
						CArtiBodyTree::Destroy(node);
					}
				};

			CArtiBodyTree::TraverseDFS(rootTrim, OnEnterBodyTrim, OnLeaveBodyTrim);

			CFile2ArtiBodyRef file_src(src, rootTrim);
			int n_frames = file_src.frames();
			CArtiBodyRef2File file_dst(rootTrim, n_frames);

			for (int i_frame = 0; i_frame < n_frames; i_frame ++)
			{
				file_src.PoseBody<false>(i_frame);
				file_dst.UpdateMotion(i_frame);
			}

			file_dst.WriteBvhFile(dst);
		}
	}
	catch(const std::string &exp)
	{
		LOGIKVarErr(LogInfoCharPtr, exp.c_str());
		ret = false;
	}

	return ret;
}
