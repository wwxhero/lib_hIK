#include "pch.h"
#include <queue>
#include <list>
#include "conf_mopipe.h"
#include "tinyxml.h"
#include "handle_helper.hpp"
#include "ik_logger.h"
#include <sstream>

class CConfMoPipe
{
private:
	class B_ScaleEx : public B_Scale
	{
	public:
		B_ScaleEx(const char* name, float x, float y, float z)
		{
			std::string strname_c(name);
			std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
			m_boneName_w = converter.from_bytes(strname_c);
			bone_name = m_boneName_w.c_str();
			scaleX = x;
			scaleY = y;
			scaleZ = z;
		}

		~B_ScaleEx()
		{
		}

		void AllocCopyTo(B_Scale* dst)
		{
			dst->bone_name = new wchar_t[m_boneName_w.length() + 1];
			wcscpy((wchar_t *)dst->bone_name, m_boneName_w.c_str());
			dst->scaleX = scaleX;
			dst->scaleY = scaleY;
			dst->scaleZ = scaleZ;
		}

		static void FreeCopy(B_Scale scale)
		{
			delete [] scale.bone_name;
		}
	private:
		std::wstring m_boneName_w;
	};

	class PairNames
	{
	public:
		PairNames(const char* j_from, const char* j_to)
			: m_from(j_from)
			, m_to(j_to)
		{
		}
		void AllocCopyTo(const wchar_t* names[2])
		{
			const std::string* m_names[2] = {&m_from, &m_to};
			std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
			for (int i = 0; i < 2; i ++)
			{
				std::wstring name_i = converter.from_bytes(*m_names[i]);
				wchar_t* temp = new wchar_t[name_i.length() + 1];
				wcscpy(temp, name_i.c_str());
				names[i] = temp;
			}
		}
		static void FreeCopy(const wchar_t* names[2])
		{
			delete [] names[0];
			delete [] names[1];
		}
	public:
		const std::string m_from;
		const std::string m_to;
	};

	class Name
	{
	public:
		Name() {}
		Name(const char* name)
		{
			std::string strname_c(name);
			std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
			m_str = converter.from_bytes(strname_c);
		}
		void AllocCopyTo(const wchar_t* *a_name)
		{
			wchar_t* name = new wchar_t[m_str.length() + 1];
			wcscpy(name, m_str.c_str());
			*a_name = name;
		}
		static void FreeCopy(const wchar_t* a_name)
		{
			delete [] a_name;
		}
	private:
		std::wstring m_str;
	};

private:

	CConfMoPipe()
	{
	}
	~CConfMoPipe()
	{
	}

	inline void copy_3x3(const Real src[3][3], Real dst[3][3])
	{
		for (int i = 0; i < 3; i ++)
		{
			for (int j = 0; j < 3; j ++)
			{
				dst[i][j] = src[i][j];
			}
		}
	}
public:

	bool AllocRCScale(B_Scale* bscales[2], int n_scales[2])
	{
		for (int idx = 0; idx < 2; idx ++)
		{
			int n_scales_i = (int)m_lstScales[idx].size();
			auto arr_scales_i = new B_Scale[n_scales_i];
			int i_scale = 0;
			for (auto scale_i : m_lstScales[idx])
			{
				scale_i.AllocCopyTo(&arr_scales_i[i_scale ++]);
			}
			bscales[idx] = arr_scales_i;
			n_scales[idx] = n_scales_i;
		}
		return true;
	}

	static void FreeRCScale(B_Scale* bscales[2], int n_scales[2])
	{
		for (int idx = 0; idx < 2; idx ++)
		{
			auto bscales_i = bscales[idx];
			auto n_scales_i = n_scales[idx];
			for (int i_scale = 0; i_scale < n_scales_i; i_scale ++)
				B_ScaleEx::FreeCopy(bscales_i[i_scale]);
			delete [] bscales_i;
		}

	}

	bool GetPipeMTX(Real m[3][3])
	{
		copy_3x3(m_mtxf2t, m);
		return true;
	}

	int AllocMOPIPEPairs(const wchar_t* (**a_match)[2])
	{
		int n_pairs = (int)m_lstPairs.size();
		const wchar_t* (*match)[2] = (const wchar_t* (*)[2])malloc(sizeof(const wchar_t*) * n_pairs * 2);
		int i_pair = 0;
		for (auto pair : m_lstPairs)
			pair.AllocCopyTo(match[i_pair ++]);
		*a_match = match;
		return n_pairs;
	}

	static void FreeMOPIPEPairs(const wchar_t* (*match)[2], int n_match)
	{
		for (int i_match = 0; i_match < n_match; i_match ++)
			PairNames::FreeCopy(match[i_match]);
		free(match);
	}

	bool AllocENDEFFNames(const wchar_t * *a_names[2], int n_names[2])
	{
		for (int idx = 0; idx < 2; idx ++)
		{
			int n_targets = (int)m_lsttargetnames[idx].size();
			const wchar_t** names_i = (const wchar_t **)malloc(sizeof(const wchar_t*) * n_targets);
			int i_name = 0;
			for (auto name : m_lsttargetnames[idx])
			{
				name.AllocCopyTo(&names_i[i_name ++]);
			}
			a_names[idx] = names_i;
			n_names[idx] = n_targets;
		}
		return true;
	}

	static void FreeEndEFFNames(const wchar_t* *a_names[2], int n_names[2])
	{
		for (int idx = 0; idx < 2; idx ++)
		{
			const wchar_t** names_i = a_names[idx];
			int n_names_i = n_names[idx];
			for (int i_name = 0; i_name < n_names_i; i_name ++)
				Name::FreeCopy(names_i[i_name]);
			free(names_i);
		}
	}

	bool AllocFileNames(const wchar_t* filenames[2])
	{
		m_filenames[0].AllocCopyTo(&filenames[0]);
		m_filenames[1].AllocCopyTo(&filenames[1]);
		return true;
	}

	static void FreeFileNames(const wchar_t* filenames[2])
	{
		Name::FreeCopy(filenames[0]);
		Name::FreeCopy(filenames[1]);
	}

// for internal use functions:

	void AddScale(const char* name, float x, float y, float z, bool is_src)
	{
		B_ScaleEx scale(name, x, y, z);
		int idx = (is_src ? 0 : 1);
		m_lstScales[idx].push_back(std::move(scale));
	}

	void UpdateMTX(const float m[3][3])
	{
		copy_3x3(m, m_mtxf2t);
	}

	void AddPair(const char* j_from, const char* j_to)
	{
		PairNames pair(j_from, j_to);
		m_lstPairs.push_back(std::move(pair));
	}

	void AddEndEFF(const char* target_name, bool is_src)
	{
		Name name(target_name);
		int idx = (is_src ? 0 : 1);
		m_lsttargetnames[idx].push_back(std::move(name));
	}

	void SetFileName(const char* file_name, bool is_src)
	{
		Name name(file_name);
		int idx = (is_src ? 0 : 1);
		m_filenames[idx] = name;
	}

	static CConfMoPipe* load(const TiXmlDocument* doc);
	static void unload(CConfMoPipe* loader);
private:
	template<typename LAMaccess>
	static bool TraverseBFS_XML_tree(const TiXmlNode* root, LAMaccess OnXmlNode)
	{
		std::queue<const TiXmlNode*> bfs_q;
		bfs_q.push(root);
		bool keep_traversing = true;
		while (!bfs_q.empty() && keep_traversing)
		{
			const TiXmlNode* node = bfs_q.front();
			bfs_q.pop();
			keep_traversing = OnXmlNode(node);
			for (auto child = node->FirstChild()
				; NULL != child && keep_traversing
				; child = child->NextSibling())
				bfs_q.push(child);
		}
		return keep_traversing;
	}

private:
	std::list<B_ScaleEx> m_lstScales[2];
	Real m_mtxf2t[3][3];
	std::list<PairNames> m_lstPairs;
	std::list<Name> m_lsttargetnames[2];
	Name m_filenames[2];
};

CConfMoPipe* CConfMoPipe::load(const TiXmlDocument* doc)
{
	assert(doc->Type() == TiXmlNode::DOCUMENT);
	CConfMoPipe* pConf = new CConfMoPipe();
	auto Is_in_Source = [] (const TiXmlNode* node) -> bool
		{
			auto node_m = node->Parent();
			assert(NULL != node_m);
			bool is_src = ("Source" == node_m->ValueStr());
			bool is_dst = ("Destination" == node_m->ValueStr());
			assert(is_src || is_dst);
			return is_src;
		};
	auto OnTraverXmlNode = [pConf, Is_in_Source] (const TiXmlNode* node) -> bool
		{
			bool ret = true;
			bool is_a_source = false;
			bool is_a_desti = false;
			if (TiXmlNode::ELEMENT == node->Type())
			{
				auto name = node->ValueStr();
				const TiXmlElement* ele = node->ToElement();
				if ("Scale" == name)
				{
					const char* b_name = ele->Attribute("b_name");
					bool valid_name = (NULL != b_name);
					float x, y, z;
					bool valid_x_scale = (TIXML_SUCCESS == ele->QueryFloatAttribute("x", &x));
					bool valid_y_scale = (TIXML_SUCCESS == ele->QueryFloatAttribute("y", &y));
					bool valid_z_scale = (TIXML_SUCCESS == ele->QueryFloatAttribute("z", &z));
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
						pConf->AddScale(b_name, x, y, z, Is_in_Source(node));
					}
				}
				else if("MotionPipe" == name)
				{
					const char* m_name[3][3] = {
						{"m11", "m12", "m13"},
						{"m21", "m22", "m23"},
						{"m31", "m32", "m33"},
					};
					float m_value[3][3] = {0};
					bool all_entry_ij = true;
					for (int i_r = 0; i_r < 3 && all_entry_ij; i_r ++)
					{
						for (int i_c = 0; i_c < 3 && all_entry_ij; i_c ++)
						{
							all_entry_ij = (TIXML_SUCCESS == ele->QueryFloatAttribute(m_name[i_r][i_c], &m_value[i_r][i_c]));
						}
					}
					ret = all_entry_ij;
					IKAssert(all_entry_ij);
					if (ret)
					{
						pConf->UpdateMTX(m_value);
					}
					else
					{
						LOGIKVar(LogInfoFloat3x3_m, m_value);
					}
				}
				else if("Pair" == name)
				{
					const char* j_from = ele->Attribute("from");
					const char* j_to = ele->Attribute("to");
					bool valid_pair = (NULL != j_from && NULL != j_to);
					IKAssert(valid_pair);
					ret = valid_pair;
					if (valid_pair)
						pConf->AddPair(j_from, j_to);
				}
				else if("EndEEF" == name)
				{
					const char* target_name = ele->Attribute("b_name");
					bool valid_target = (NULL != target_name);
					IKAssert(valid_target);
					ret = valid_target;
					if (valid_target)
						pConf->AddEndEFF(target_name, Is_in_Source(node));
				}
				else if((is_a_source = ("Source" == name))
					|| (is_a_desti = ("Destination" == name)))
				{
					const char* filename = ele->Attribute("file");
					pConf->SetFileName(filename, is_a_source);
				}
			}
			return ret;
		};

	if (TraverseBFS_XML_tree(doc, OnTraverXmlNode))
		return pConf;
	else
	{
		delete pConf;
		return NULL;
	}
}

void CConfMoPipe::unload(CConfMoPipe* loader)
{
	delete loader;
}


HCONFMOPIPE init_conf_mopipe(HCONF conf)
{
	TiXmlDocument* doc = CAST_2PCONF(conf);
	if (NULL != doc)
	{
		CConfMoPipe* conf_fkrc_loader = CConfMoPipe::load(doc);
		return CAST_2HCONFFKRC(conf_fkrc_loader);
	}
	else
		return H_INVALID;
}

void uninit_conf_mopipe(HCONFMOPIPE conf)
{
	CConfMoPipe* conf_fkrc_loader = CAST_2PCONFFKRC(conf);
	CConfMoPipe::unload(conf_fkrc_loader);
}

bool load_scale(HCONFMOPIPE conf, B_Scale* bscales[2], int n_scales[2])
{
	CConfMoPipe* conf_fkrc_loader = CAST_2PCONFFKRC(conf);
	if (NULL != conf_fkrc_loader)
		return conf_fkrc_loader->AllocRCScale(bscales, n_scales);
	else
		return false;
}

void free_scale(B_Scale* bscales[2], int n_scales[2])
{
	CConfMoPipe::FreeRCScale(bscales, n_scales);
}

bool get_mopipe_mtx(HCONFMOPIPE conf, Real m[3][3])
{
	CConfMoPipe* conf_fkrc_loader = CAST_2PCONFFKRC(conf);
	if (NULL != conf_fkrc_loader)
		return conf_fkrc_loader->GetPipeMTX(m);
	else
		return false;
}

int load_mopipe_pairs(HCONFMOPIPE conf, const wchar_t* (**match)[2])
{
	CConfMoPipe* conf_fkrc_loader = CAST_2PCONFFKRC(conf);
	if (NULL != conf_fkrc_loader)
		return conf_fkrc_loader->AllocMOPIPEPairs(match);
	else
		return -1;
}

void free_mopipe_pairs(const wchar_t* (*match)[2], int n_match)
{
	CConfMoPipe::FreeMOPIPEPairs(match, n_match);
}

bool load_endeff_names(HCONFMOPIPE conf, const wchar_t ** names[2], int n_names[2])
{
	CConfMoPipe* conf_fkrc_loader = CAST_2PCONFFKRC(conf);
	if (NULL != conf_fkrc_loader)
		return conf_fkrc_loader->AllocENDEFFNames(names, n_names);
	else
		return false;
}

void free_endeff_names(const wchar_t** names[2], int n_names[2])
{
	CConfMoPipe::FreeEndEFFNames(names, n_names);
}

bool load_file_names(HCONFMOPIPE conf, const wchar_t* filenames[2])
{
	CConfMoPipe* conf_fkrc_loader = CAST_2PCONFFKRC(conf);
	if (NULL != conf_fkrc_loader)
		return conf_fkrc_loader->AllocFileNames(filenames);
	else
		return false;
}

void free_file_names(const wchar_t* filenames[2])
{
	CConfMoPipe::FreeFileNames(filenames);
}