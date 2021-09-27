#include "pch.h"
#include <utility>
#include <filesystem>
#include "ik_logger.h"
#include "MotionPipeConf.hpp"
#include "tinyxml.h"

extern HMODULE g_Module;

namespace CONF
{
	std::wstring GetModuleDir()
	{
	    unsigned int n_path_spec = MAX_PATH;
	    wchar_t* path = (wchar_t*)malloc(n_path_spec * sizeof(wchar_t));
	    unsigned int n_path = 0;
	    while ( (n_path = GetModuleFileNameW(g_Module, path, n_path_spec)) > n_path_spec )
	    {
	        n_path_spec = n_path;
	        realloc(path, n_path_spec * sizeof(wchar_t));
	    }
	    std::experimental::filesystem::path fullPath(path);
	    std::wstring dir = fullPath.parent_path().c_str();
	    free(path);
	    LOGIKVar(LogInfoWCharPtr, dir.c_str());
	    return dir;
	}

	class TiXMLHelper
	{
	public:
		static int QueryRealAttribute(const TiXmlElement* ele, const char* name, Real* _value );
	};

	int TiXMLHelper::QueryRealAttribute(const TiXmlElement* ele, const char* name, Real* _value )
	{
		double d;
		int result = ele->QueryDoubleAttribute( name, &d );
		if ( result == TIXML_SUCCESS ) {
			*_value = (Real)d;
		}
		return result;
	}

	////////////////////////////////////////////////////////////////////////////////////////////
	//CIKChainConf:

	CIKChainConf::CIKChainConf(const char* a_eef_name
							, int a_len
							, CIKChain::Algor a_algor
							, Real a_weight_p
							, Real a_weight_r
							, int a_n_iter
							, const char* a_P_Graph)
		: eef(a_eef_name)
		, len(a_len)
		, algor(a_algor)
		, weight_p(a_weight_p)
		, weight_r(a_weight_r)
		, n_iter(a_n_iter)
		, P_Graph(a_P_Graph)
	{
	}

	CIKChainConf::CIKChainConf(const char* a_eef_name
							, int a_len
							, CIKChain::Algor a_algor
							, Real a_up[3])
		: eef(a_eef_name)
		, len(a_len)
		, algor(a_algor)
		, up {a_up[0], a_up[1], a_up[2]}
	{

	}

	CIKChainConf::CIKChainConf(const CIKChainConf& src)
	{
		eef = src.eef;
		len = src.len;
		algor = src.algor;
		if (NumericalAlgor(src.algor))
		{
			weight_p = src.weight_p;
			weight_r = src.weight_r;
			n_iter = src.n_iter;
		}
		else if(CIKChain::Proj == src.algor)
		{
			memcpy(up, src.up, 3 * sizeof(Real));
		}
		P_Graph = src.P_Graph;

	}

	CIKChainConf::~CIKChainConf()
	{
	}

	void CIKChainConf::AddJoint(const char* attri_values[5])
	{
		const char* name = attri_values[0];
		IK_QSegment::Type type = (NULL == attri_values[1])
							? IK_QSegment::R_xyz
							: IK_QSegment::to_Type(attri_values[1]);
		Real dexterity[] = {
			(NULL == attri_values[2]) ? 1 : (Real)atof(attri_values[2]),
			(NULL == attri_values[3]) ? 1 : (Real)atof(attri_values[3]),
			(NULL == attri_values[4]) ? 1 : (Real)atof(attri_values[4]),
		};
		CJointConf joint_conf(name, type, dexterity);
		Joints.push_back(joint_conf);
	}

#ifdef _DEBUG
	void CIKChainConf::Dump_Dbg() const
	{
		LOGIKVar(LogInfoCharPtr, eef.c_str());
		LOGIKVar(LogInfoInt, len);
		LOGIKVar(LogInfoCharPtr, CIKChain::from_Algor(algor));
		LOGIKVar(LogInfoReal, weight_p);
		LOGIKVar(LogInfoReal, weight_r);
		LOGIKVar(LogInfoInt, n_iter);
		LOGIKVar(LogInfoCharPtr, P_Graph.c_str());

		for (const CJointConf& joint_conf : Joints)
		{
			joint_conf.Dump_Dbg();
		}
	}
#endif

	////////////////////////////////////////////////////////////////////////////////////////////
	//CBodyConf:

	int CBodyConf::Scale_alloc(B_Scale* &scales) const
	{
		int n_scales = (int)m_scales.size();
		scales = new B_Scale[n_scales];
		for (int i_scale = 0; i_scale < n_scales; i_scale++)
		{
			m_scales[i_scale].AllocCopyTo(&scales[i_scale]);
		}
		return n_scales;
	}

	void CBodyConf::Scale_free(B_Scale* scales, int n_scales)
	{
		for (int i_scale = 0; i_scale < n_scales; i_scale++)
		{
			B_ScaleEx::FreeCopy(scales[i_scale]);
		}
		delete[] scales;
	}

	void CBodyConf::AddScale(const char* name, Real x, Real y, Real z)
	{
		B_ScaleEx bs(name, x, y, z);
		m_scales.push_back(bs);
	}


	int CBodyConf::Targets_alloc(const wchar_t** &nameTargets) const
	{
		int n_targets = (int)m_targets.size();
		nameTargets = (const wchar_t**)malloc(n_targets * sizeof(const wchar_t*));

		for (int i_eef = 0; i_eef < n_targets; i_eef++)
		{
			m_targets[i_eef].AllocCopyTo(&nameTargets[i_eef]);
		}
		return n_targets;
	}

	void CBodyConf::Targets_free(const wchar_t** nameTargets, int n_targets)
	{
		for (int i_eef = 0; i_eef < n_targets; i_eef++)
		{
			Name::FreeCopy(nameTargets[i_eef]);
		}
		free(nameTargets);
	}

	void CBodyConf::AddTarget(const char* name)
	{
		Name target(name);
		m_targets.push_back(target);
	}

	const wchar_t* CBodyConf::file_w() const
	{
		return m_fileName_w.c_str();
	}

	const char* CBodyConf::file_c() const
	{
		return m_fileName_c.c_str();
	}

	void CBodyConf::SetFileName(const char* filename)
	{
		m_fileName_c = filename;
		std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
		m_fileName_w = converter.from_bytes(filename);
	}

#ifdef _DEBUG
	void CBodyConf::Dump_Dbg() const
	{
		LOGIKVar(LogInfoWCharPtr, m_fileName_w.c_str());

		B_Scale* scales = NULL;
		int n_scales = Scale_alloc(scales);
		for (int i_scale = 0; i_scale < n_scales; i_scale ++)
		{
			const B_Scale& scale_i = scales[i_scale];
			LOGIKVar(LogInfoWCharPtr, scale_i.bone_name);
			LOGIKVar(LogInfoReal, scale_i.scaleX);
			LOGIKVar(LogInfoReal, scale_i.scaleY);
			LOGIKVar(LogInfoReal, scale_i.scaleZ);
		}
		CBodyConf::Scale_free(scales, n_scales);

		const wchar_t **nameTargets = NULL;
		int n_Targets = Targets_alloc(nameTargets);
		for (int i_target = 0; i_target < n_Targets; i_target ++)
		{
			LOGIKVar(LogInfoWCharPtr, nameTargets[i_target]);
		}
		CBodyConf::Targets_free(nameTargets, n_Targets);

		for (const CIKChainConf& ikchain_conf : IK_Chains)
		{
			ikchain_conf.Dump_Dbg();
		}
	}
#endif

	BODY_TYPE CBodyConf::type() const
	{
		std::experimental::filesystem::path relpath(file_w());
		std::wstring ext = relpath.extension().c_str();
		const wchar_t* body_types_str[] = {
			L".fbx",	L".bvh",	L".htr"
		};

		const BODY_TYPE body_types[] = {
			fbx, bvh, htr
		};

		for (int i_body_type = 0
			; i_body_type < sizeof(body_types_str)/sizeof(const wchar_t*)
			; i_body_type ++)
		{
			if (ext == body_types_str[i_body_type])
				return (BODY_TYPE)body_types[i_body_type];
		}

		return undef;
	}

	void CBodyConf::AddIKChain(const char* eef_name
						, int len
						, CIKChain::Algor algor
						, Real weight_p
						, Real weight_r
						, int n_iter
						, const char* P_Graph)
	{
		CIKChainConf chain_conf(eef_name, len, algor, weight_p, weight_r, n_iter, P_Graph);
		int i_new_chain = (int)IK_Chains.size();
		IK_Chains.push_back(chain_conf);
		m_name2chainIdx[eef_name] = i_new_chain;
	}

	void CBodyConf::AddIKChain(const char* eef_name
						, int len
						, CIKChain::Algor algor
						, Real up[3])
	{
		CIKChainConf chain_conf(eef_name, len, algor, up);
		int i_new_chain = (int)IK_Chains.size();
		IK_Chains.push_back(chain_conf);
		m_name2chainIdx[eef_name] = i_new_chain;
	}

	CIKChainConf* CBodyConf::GetIKChain(const char* eef_name)
	{
		auto it = m_name2chainIdx.find(eef_name);
		if (m_name2chainIdx.end() != it)
		{
			int i_chain = it->second;
			return &IK_Chains[i_chain];
		}
		else
			return NULL;
	}

	int CBodyConf::Name2IKChainIdx(const char* eef_name) const
	{
		auto it = m_name2chainIdx.find(eef_name);
		if (m_name2chainIdx.end() != it)
		{
			return it->second;
		}
		else
			return -1;
	}

	////////////////////////////////////////////////////////////////////////////////////////////
	// CPairsConf:

	int CPairsConf::Data_alloc(int i_body, const wchar_t** &namesOnPair) const
	{
		int n_pairs = (int)m_pairs.size();
		namesOnPair = (const wchar_t**)malloc(n_pairs * sizeof(const wchar_t*));
		for (int i_pair = 0; i_pair < n_pairs; i_pair ++)
		{
			const Name* pair_i [] = { &m_pairs[i_pair].first, &m_pairs[i_pair].second };
			pair_i[i_body]->AllocCopyTo(&namesOnPair[i_pair]);
		}
		return n_pairs;
	}

	void CPairsConf::Data_free(const wchar_t** namesOnPair, int n_pairs)
	{
		for (int i_pair = 0; i_pair < n_pairs; i_pair ++)
		{
			Name::FreeCopy(namesOnPair[i_pair]);
		}
		free(namesOnPair);
	}

	int CPairsConf::Data_alloc(const wchar_t* (**a_matches)[2]) const
	{
		int n_pairs = (int)m_pairs.size();
		const wchar_t* (*matches)[2] = (const wchar_t* (*)[2])malloc(n_pairs * sizeof(const wchar_t*) * 2);
		for (int i_pair = 0; i_pair < n_pairs; i_pair ++)
		{
			m_pairs[i_pair].first.AllocCopyTo(&(matches[i_pair][0]));
			m_pairs[i_pair].second.AllocCopyTo(&(matches[i_pair][1]));
		}
		*a_matches = matches;
		return n_pairs;
	}

	void CPairsConf::Data_free(const wchar_t* (*matches)[2], int n_pairs)
	{
		for (int i_pair = 0; i_pair < n_pairs; i_pair ++)
		{
			Name::FreeCopy(matches[i_pair][0]);
			Name::FreeCopy(matches[i_pair][1]);
		}
		free(matches);
	}

	void CPairsConf::Map(std::map<std::wstring, std::wstring>& name2name, bool forward)
	{
		int src;
		int dst;
		if (forward)
		{
			src = 0; dst = 1;
		}
		else
		{
			src = 1; dst = 0;
		}
		for (const auto& pair : m_pairs)
		{
			const Name* names[] = { &pair.first, &pair.second };
			name2name[*names[src]] = *names[dst];
		}
	}

	void CPairsConf::Add(const char* from, const char* to)
	{
		auto pair = std::make_pair(Name(from)
								, Name(to));
		m_pairs.push_back(pair);
	}

#ifdef _DEBUG
	void CPairsConf::Dump_Dbg() const
	{
		const wchar_t** namesOnPair = NULL;
		for (int i_body = 0; i_body < 2; i_body ++)
		{
			int n_pairs = Data_alloc(i_body, namesOnPair);
			for (int i_pair = 0; i_pair < n_pairs; i_pair ++)
			{
				const wchar_t* name_i = namesOnPair[i_pair];
				LOGIKVar(LogInfoWCharPtr, name_i);
			}
			CPairsConf::Data_free(namesOnPair, n_pairs);
		}

		const wchar_t* (*matches)[2] = NULL;
		int n_matches = Data_alloc(&matches);
		for (int i_match = 0; i_match < n_matches; i_match ++)
		{
			const wchar_t* name_i_0 = matches[i_match][0];
			const wchar_t* name_i_1 = matches[i_match][1];
			LOGIKVar(LogInfoWCharPtr, name_i_0);
			LOGIKVar(LogInfoWCharPtr, name_i_1);
		}
		CPairsConf::Data_free(matches, n_matches);

	}
#endif

	////////////////////////////////////////////////////////////////////////////////////////////
	// CMotionPipeConf:
	#define SETIDENTITY_3X3(mtx)\
		for (int i_r = 0; i_r < 3; i_r ++)\
			for (int i_c = 0; i_c < 3; i_c ++)\
				mtx[i_r][i_c] = (i_r == i_c);



	CMotionPipeConf* CMotionPipeConf::Load(const wchar_t* confXML)
	{
		std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
		std::string path_c = converter.to_bytes(confXML);
		TiXmlDocument* doc = new TiXmlDocument(path_c);
		CMotionPipeConf* conf = NULL;
		bool loadOkay = doc->LoadFile();
		if (loadOkay)
		{
			conf = new CMotionPipeConf();
			if (!conf->Initialize(doc))
			{
				delete conf;
				conf = NULL;
			}
		}
		delete doc;
		return conf;
	}

	void CMotionPipeConf::UnLoad(CMotionPipeConf* conf)
	{
		delete conf;
	}

	CMotionPipeConf::CMotionPipeConf()
		: sync(CMoNode::unknown)
	{
		SETIDENTITY_3X3(m);
		SETIDENTITY_3X3(m_inv);
	}

	CMotionPipeConf::~CMotionPipeConf()
	{

	}

	bool CMotionPipeConf::Initialize(TiXmlDocument* doc)
	{
		assert(doc->Type() == TiXmlNode::DOCUMENT);

		auto I_Body = [](const TiXmlNode* node) -> int
		{
			do
			{
				assert(NULL != node);
				auto v_str = node->ValueStr();
				bool is_src = ("Source" == v_str);
				bool is_dst = ("Destination" == v_str);
				if (is_src)
					return 0;
				else if (is_dst)
					return 1;
			} while (node = node->Parent());

			assert(0);
			return -1;
		};

		auto P_Chain = [this, I_Body](const TiXmlNode* node) -> CIKChainConf*
		{
			const char* eef_name = NULL;
			do
			{
				assert(NULL != node);
				auto v_str = node->ValueStr();
				bool is_chain = ("IK_Chain" == v_str);
				if (is_chain)
				{
					const TiXmlElement* ele = node->ToElement();
					eef_name = ele->Attribute("eef");
				}
			} while (NULL != (node = node->Parent())
				&& NULL == eef_name);

			if (NULL != eef_name)
			{
				CBodyConf* body_confs[] = {&Source, &Destination};
				return body_confs[I_Body(node)]->GetIKChain(eef_name);
			}
			else
				return NULL;
		};

		auto OnTraverXmlNode = [this, I_Body, P_Chain](const TiXmlNode* node) -> bool
		{
			bool ret = true;
			bool is_a_source = false;
			bool is_a_desti = false;
			CBodyConf* body_confs[] = { &Source, &Destination };
			if (TiXmlNode::ELEMENT == node->Type())
			{
				auto name = node->ValueStr();
				const TiXmlElement* ele = node->ToElement();
				if ("Scale" == name)
				{
					const char* b_name = ele->Attribute("b_name");
					bool valid_name = (NULL != b_name);
					Real x, y, z;
					bool valid_x_scale = (TIXML_SUCCESS == TiXMLHelper::QueryRealAttribute(ele, "x", &x));
					bool valid_y_scale = (TIXML_SUCCESS == TiXMLHelper::QueryRealAttribute(ele, "y", &y));
					bool valid_z_scale = (TIXML_SUCCESS == TiXMLHelper::QueryRealAttribute(ele, "z", &z));
					ret = (valid_name
						&& valid_x_scale
						&& valid_y_scale
						&& valid_z_scale);
					IKAssert(valid_name);
					IKAssert(valid_x_scale);
					IKAssert(valid_y_scale);
					IKAssert(valid_z_scale);
					if (ret)
					{
						body_confs[I_Body(node)]->AddScale(b_name, x, y, z);
					}
				}
				else if ("MotionPipe" == name)
				{
					const char* sync_type = ele->Attribute("sync");
					sync = CMoNode::to_RETAR_TYPE(sync_type);

					const char* m_name[3][3] = {
						{"m11", "m12", "m13"},
						{"m21", "m22", "m23"},
						{"m31", "m32", "m33"},
					};

					bool all_entry_ij = true;
					for (int i_r = 0; i_r < 3 && all_entry_ij; i_r++)
					{
						for (int i_c = 0; i_c < 3 && all_entry_ij; i_c++)
						{
							all_entry_ij = (TIXML_SUCCESS == TiXMLHelper::QueryRealAttribute(ele, m_name[i_r][i_c], &m[i_r][i_c]));
						}
					}
					ret = all_entry_ij;
					IKAssert(all_entry_ij);
					if (ret)
					{
						LOGIKVar(LogInfoReal3x3_m, m);
						Eigen::Matrix3r src2dst_w;
						src2dst_w << m[0][0], m[0][1], m[0][2],
									 m[1][0], m[1][1], m[1][2],
									 m[2][0], m[2][1], m[2][2];
						Eigen::Matrix3r dst2src_w = src2dst_w.inverse();
						for (int i_r = 0; i_r < 3 && all_entry_ij; i_r++)
						{
							for (int i_c = 0; i_c < 3 && all_entry_ij; i_c++)
							{
								m_inv[i_r][i_c] = dst2src_w(i_r, i_c);
							}
						}
					}
					else
					{
						SETIDENTITY_3X3(m);
						SETIDENTITY_3X3(m_inv);
					}
				}
				else if ("Pair" == name)
				{
					const char* j_from = ele->Attribute("from");
					const char* j_to = ele->Attribute("to");
					bool valid_pair = (NULL != j_from && NULL != j_to);
					IKAssert(valid_pair);
					ret = valid_pair;
					if (valid_pair)
						Pair.Add(j_from, j_to);
				}
				else if ("Target" == name)
				{
					const char* target_name = ele->Attribute("b_name");
					bool valid_target = (NULL != target_name);
					IKAssert(valid_target);
					ret = valid_target;
					if (valid_target)
						body_confs[I_Body(node)]->AddTarget(target_name);
				}
				else if ((is_a_source = ("Source" == name))
					|| (is_a_desti = ("Destination" == name)))
				{
					const char* filename = ele->Attribute("file");
					body_confs[I_Body(node)]->SetFileName(filename);
				}
				else if("IK_Chain" == name)
				{
					const char* eef_name = ele->Attribute("eef");
					int len = -1;
					bool valid_len = (TIXML_SUCCESS == ele->QueryIntAttribute("len", &len));
					CIKChain::Algor algor = CIKChain::Unknown;
					const char* algor_str = NULL;
					bool valid_algor = (NULL != (algor_str = ele->Attribute("algor"))
									&& CIKChain::Unknown != (algor = CIKChain::to_Algor(algor_str)));
					Real weight_p = 0;
					bool valid_weight_p = (TIXML_SUCCESS == TiXMLHelper::QueryRealAttribute(ele, "weight_p", &weight_p));
					Real weight_r = 0;
					bool valid_weight_r = (TIXML_SUCCESS == TiXMLHelper::QueryRealAttribute(ele, "weight_r", &weight_r));
					Real up[3];
					bool valid_up =   (TIXML_SUCCESS == TiXMLHelper::QueryRealAttribute(ele, "up_x", &up[0])
									&& TIXML_SUCCESS == TiXMLHelper::QueryRealAttribute(ele, "up_y", &up[1])
									&& TIXML_SUCCESS == TiXMLHelper::QueryRealAttribute(ele, "up_z", &up[2]));

					IKAssert(valid_len);
					IKAssert(valid_algor);
					bool numerical_algor = NumericalAlgor(algor);
					IKAssert(!numerical_algor || (valid_weight_p && valid_weight_r));
					IKAssert(CIKChain::Proj != algor || valid_up);

					int n_iter = 20;
					if (TIXML_SUCCESS != ele->QueryIntAttribute("n_iter", &n_iter))
						n_iter = 20;

					const char* P_Graph = ele->Attribute("P_Graph");

					if (numerical_algor)
						body_confs[I_Body(node)]->AddIKChain(eef_name
														, len
														, algor
														, weight_p
														, weight_r
														, n_iter
														, NULL != P_Graph ? P_Graph : "");
					else
						body_confs[I_Body(node)]->AddIKChain(eef_name
														, len
														, algor
														, up);
				}
				else if("Joint" == name)
				{
					struct
					{
						const char* str;
						bool is_optional;
					} names_attri[] = {
						  { "name",			false }		// 0
						, { "type",			true }		// 1
						, { "Dexterity_x",	true }		// 2
						, { "Dexterity_y",	true }		// 3
						, { "Dexterity_z",	true }		// 4
					};
					const char* value_attri[] = {
						  ele->Attribute(names_attri[0].str)
						, ele->Attribute(names_attri[1].str)
						, ele->Attribute(names_attri[2].str)
						, ele->Attribute(names_attri[3].str)
						, ele->Attribute(names_attri[4].str)
					};

					bool value_valid = true;
					for (int i_value = 0
						; i_value < sizeof(value_attri)/sizeof(const char*)
							&& value_valid
						; i_value ++)
						value_valid = (NULL != value_attri[i_value]
									|| names_attri[i_value].is_optional);
					IKAssert(value_valid);

					CIKChainConf* chain_conf = P_Chain(node);
					IKAssert(NULL != chain_conf);
					chain_conf->AddJoint(value_attri);
				}
			}
			return ret;
		};

		return (TraverseBFS_XML_tree(doc, OnTraverXmlNode));
	}
#ifdef _DEBUG
	void CMotionPipeConf::Dump_Dbg() const
	{
		LOGIKVar(LogInfoCharPtr, CMoNode::from_RETAR_TYPE(sync));
		LOGIKVar(LogInfoReal3x3_m, m);
		Source.Dump_Dbg();
		Destination.Dump_Dbg();
		Pair.Dump_Dbg();
	}
#endif
	#undef SETIDENTITY_3X3
}