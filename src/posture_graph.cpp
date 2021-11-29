#include "pch.h"
#include <fstream>
#include "handle_helper.hpp"
#include "Math.hpp"
#include "posture_graph.h"
#include "ArtiBody.hpp"
#include "ArtiBodyFile.hpp"
#include "MotionPipeConf.hpp"
#include "IKGroupTree.hpp"
#include "filesystem_helper.hpp"
#include "PostureGraph.hpp"
#include "PostureGraph_helper.hpp"

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

		CPGTheta theta(path_htr);
		IErrorTB* err_out = IErrorTB::Factory::CreateHOMO(theta, interests_conf->Joints);
		err_out->Alloc(err_tb);
		IErrorTB::Factory::Release(err_out);
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

		CPGTheta theta_0(pg_theta_0);
		CPGTheta theta_1(pg_theta_1);
		int n_theta_0 = theta_0.N_Theta();
		int n_theta_1 = theta_1.N_Theta();
		bool merged = theta_0.Merge(theta_1);
		if (merged)
		{
			IErrorTB* err_out = IErrorTB::Factory::CreateX(theta_0, interests_conf->Joints, n_theta_0, n_theta_1);
			err_out->Alloc(err_tb);
			IErrorTB::Factory::Release(err_out);
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
	IErrorTB::Free(err_tb);
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

		CPGTheta theta(path);
		const CArtiBodyNode* body_root = theta.GetBody();
		IKAssert(NULL != body_root);
		ik_group = CIKGroupTree::Generate(body_root, *body_conf);
		if (NULL == ik_group)
		{
			err = "IK group is not created, confirm with body conf xml file!!!";
			LOGIKVarErr(LogInfoCharPtr, err.c_str());
			ok = false;
			goto EXIT;
		}

		int n_theta = theta.N_Theta();
		auto OnGroupNode = [&section, n_theta](CIKGroupNode* g_node)
			{
				if (!g_node->Empty())
				{
					CArtiBodyNode* group_root = g_node->RootBody();
					CArtiBodyRef2File* group_file = new CArtiBodyRef2File(group_root, n_theta);
					section.push_back({group_root, group_file});
				}
			};

		auto OffGroupNode = [](CIKGroupNode* g_node)
			{
			};

		CIKGroupTree::TraverseDFS(ik_group, OnGroupNode, OffGroupNode);

		for (int i_theta = 0; i_theta < n_theta; i_theta++)
		{
			theta.PoseBody<false>(i_theta);
			for (auto sec : section)
			{
				CArtiBodyTree::FK_Update<true>(sec.group_root);
				sec.group_file->UpdateMotion(i_theta);
			}
		}

		fs::path out_path_dir(dir_out);
		for (auto sec : section)
		{
			fs::path out_path(out_path_dir);
			std::string file_name(sec.group_root->GetName_c());
			if (htr == theta.GetBody()->c_type)
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

bool posture_graph_gen(const char* interests_conf_path, const char* path_htr, const char* dir_out, Real epsErr, int* n_theta_raw, int* n_theta_pg)
{
	bool ok = false;
	try
	{
		CPGTheta theta(path_htr);
		*n_theta_raw = theta.N_Theta();
		CONF::CInterestsConf* interests_conf = CONF::CInterestsConf::Load(interests_conf_path);
		if (NULL == interests_conf)
		{
			std::stringstream err;
			err << "loading " << interests_conf_path << " failed";
			LOGIKVarErr(LogInfoCharPtr, err.str().c_str());
			return false;
		}

		CPG* pg = NULL;
		if (*n_theta_raw < MAX_N_THETA_HOMO)
			pg = generate_pg_homo<CPGMatrixGen, CPGMatrixGenHelper>(theta, interests_conf->Joints, epsErr);
		else
			pg = generate_pg_homo<CPGListGen, CPGListGenHelper>(theta, interests_conf->Joints, epsErr);

		CONF::CInterestsConf::UnLoad(interests_conf);

		ok = (NULL != pg);
		if (ok)
		{
			pg->Save(dir_out);
			*n_theta_pg = pg->Theta().N_Theta();
			delete pg;
		}
	}
	catch (std::string& err)
	{
		LOGIKVarErr(LogInfoCharPtr, err.c_str());
		ok = false;
	}
	return ok;
}

HPG posture_graph_load(const char* pg_dir_0, const char* pg_name)
{
	CPG* pg = new CPG();
	if (!pg->Load(pg_dir_0, pg_name))
	{
		delete pg;
		return H_INVALID;
	}
	else
		return CAST_2HPG(pg);
}

void posture_graph_release(HPG hPG)
{
	CPG* pg = CAST_2PPG(hPG);
	delete pg;
}

HPG posture_graph_merge(HPG hpg_0, HPG hpg_1, const char* interests_conf_path, Real eps_err)
{
	try
	{
		bool ok = false;
		HPG hpg = H_INVALID;
		CPG* pg_0 = CAST_2PPG(hpg_0);
		CPG* pg_1 = CAST_2PPG(hpg_1);
		IKAssert(NULL != pg_0 && NULL != pg_1);
		int n_theta_0 = pg_0->Theta().N_Theta();
		int n_theta_1 = pg_1->Theta().N_Theta();

		CONF::CInterestsConf* interests_conf = CONF::CInterestsConf::Load(interests_conf_path);
		ok = (NULL != interests_conf);
		if (!ok)
		{
			std::stringstream err;
			err << "loading " << interests_conf_path << " failed";
			LOGIKVarErr(LogInfoCharPtr, err.str().c_str());
			return H_INVALID;
		}

		CPG* pg = NULL;
		if (n_theta_0*n_theta_1 < MAX_N_THETA_X)
			pg = generate_pg_cross<CPGMatrixGen, CPGMatrixGenHelper>(pg_0, pg_1, interests_conf->Joints, eps_err);
		else
			pg = generate_pg_cross<CPGListGen, CPGListGenHelper>(pg_0, pg_1, interests_conf->Joints, eps_err);

		CONF::CInterestsConf::UnLoad(interests_conf);
		ok = (NULL != pg);
		if (!ok)
		{
			std::string err("Generate CPG failed");
			LOGIKVarErr(LogInfoCharPtr, err.c_str());
			return H_INVALID;
		}

		hpg = CAST_2HPG(pg);
		return hpg;
	}
	catch (std::string& err)
	{
		LOGIKVarErr(LogInfoCharPtr, err.c_str());
		return H_INVALID;
	}
}

bool posture_graph_save(HPG hpg, const char* dir_out)
{
	CPG* pPG = CAST_2PPG(hpg);
	if (pPG)
	{
		pPG->Save(dir_out);
		return true;
	}
	else
		return false;
}

bool convert_pg2dot(const char* path_src, const char* path_dst)
{
	CPGRuntime filePG;
	filePG.LoadTransitions(path_src);
	filePG.SaveTransitions(path_dst, F_DOT);
	return true;
}

bool trim(const char* src, const char* dst, const char* const names_rm[], int n_names)
{
	bool ret = false;
	try
	{
		CPGTheta theta(src);
		CArtiBodyNode* rootTrim = NULL;
		ret = CArtiBodyTree::Clone(theta.GetBody(), &rootTrim);
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

			CPGThetaRuntime file_src(src, rootTrim);
			int n_theta = file_src.N_Theta();
			CArtiBodyRef2File file_dst(rootTrim, n_theta);

			for (int i_theta = 0; i_theta < n_theta; i_theta ++)
			{
				file_src.PoseBody<false>(i_theta);
				file_dst.UpdateMotion(i_theta);
			}

			CArtiBodyTree::Destroy(rootTrim);
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

int N_Theta(HPG hpg)
{
	CPG* pPG = CAST_2PPG(hpg);
	return pPG->Theta().N_Theta();
}