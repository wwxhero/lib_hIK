#include "pch.h"
#include <queue>
#include <list>
#include "fk_drv_conf.h"
#include "tinyxml.h"
#include "handle_helper.hpp"
#include "ik_logger.h"
#include <sstream>

class CConfFKRC
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

private:

	CConfFKRC()
	{
	}
	~CConfFKRC()
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

	int AllocRCScale(B_Scale** bscales)
	{
		int n_scales = (int)m_lstScales.size();
		auto arr_scales = new B_Scale[n_scales];
		int i_scale = 0;
		for (auto scale_i : m_lstScales)
		{
			scale_i.AllocCopyTo(&arr_scales[i_scale ++]);
		}
		*bscales = arr_scales;
		return n_scales;
	}

	static void FreeRCScale(B_Scale* bscales, int n_scales)
	{
		for (int i_scale = 0; i_scale < n_scales; i_scale ++)
			B_ScaleEx::FreeCopy(bscales[i_scale]);
		delete [] bscales;
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

	void AddScale(const char* name, float x, float y, float z)
	{
		B_ScaleEx scale(name, x, y, z);
		m_lstScales.push_back(std::move(scale));
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

	static CConfFKRC* load(const TiXmlDocument* doc);
	static void unload(CConfFKRC* loader);
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
	std::list<B_ScaleEx> m_lstScales;
	Real m_mtxf2t[3][3];
	std::list<PairNames> m_lstPairs;
};

CConfFKRC* CConfFKRC::load(const TiXmlDocument* doc)
{
	assert(doc->Type() == TiXmlNode::DOCUMENT);
	CConfFKRC* pConfFKRC = new CConfFKRC();
	auto OnTraverXmlNode = [pConfFKRC] (const TiXmlNode* node) -> bool
		{
			bool ret = true;
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
						pConfFKRC->AddScale(b_name, x, y, z);
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
						pConfFKRC->UpdateMTX(m_value);
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
						pConfFKRC->AddPair(j_from, j_to);
				}
			}
			return ret;
		};

	if (TraverseBFS_XML_tree(doc, OnTraverXmlNode))
		return pConfFKRC;
	else
	{
		delete pConfFKRC;
		return NULL;
	}
}

void CConfFKRC::unload(CConfFKRC* loader)
{
	delete loader;
}


HCONFFKRC init_fkrc(HCONF conf)
{
	TiXmlDocument* doc = CAST_2PCONF(conf);
	if (NULL != doc)
	{
		CConfFKRC* conf_fkrc_loader = CConfFKRC::load(doc);
		return CAST_2HCONFFKRC(conf_fkrc_loader);
	}
	else
		return H_INVALID;
}

void uninit_fkrc(HCONFFKRC conf)
{
	CConfFKRC* conf_fkrc_loader = CAST_2PCONFFKRC(conf);
	CConfFKRC::unload(conf_fkrc_loader);
}

int	load_rc_scale(HCONFFKRC conf, B_Scale** bscales)
{
	CConfFKRC* conf_fkrc_loader = CAST_2PCONFFKRC(conf);
	if (NULL != conf_fkrc_loader)
		return conf_fkrc_loader->AllocRCScale(bscales);
	else
		return -1;
}

void free_rc_scale(B_Scale* bscales, int n_scales)
{
	CConfFKRC::FreeRCScale(bscales, n_scales);
}

bool get_mopipe_mtx(HCONFFKRC conf, Real m[3][3])
{
	CConfFKRC* conf_fkrc_loader = CAST_2PCONFFKRC(conf);
	if (NULL != conf_fkrc_loader)
		return conf_fkrc_loader->GetPipeMTX(m);
	else
		return false;
}

int load_mopipe_pairs(HCONFFKRC conf, const wchar_t* (**match)[2])
{
	CConfFKRC* conf_fkrc_loader = CAST_2PCONFFKRC(conf);
	if (NULL != conf_fkrc_loader)
		return conf_fkrc_loader->AllocMOPIPEPairs(match);
	else
		return -1;
}

void free_mopipe_pairs(const wchar_t* (*match)[2], int n_match)
{
	CConfFKRC::FreeMOPIPEPairs(match, n_match);
}