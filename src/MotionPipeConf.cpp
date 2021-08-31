#include "pch.h"
#include "ik_logger.h"
#include "MotionPipeConf.hpp"
#include "tinyxml.h"

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
		: sync(unknown)
{

}

bool CMotionPipeConf::Initialize(TiXmlDocument* doc)
{
	assert(doc->Type() == TiXmlNode::DOCUMENT);

	auto I_Body = [] (const TiXmlNode* node) -> int
		{
			auto node_m = node->Parent();
			assert(NULL != node_m);
			bool is_src = ("Source" == node_m->ValueStr());
			bool is_dst = ("Destination" == node_m->ValueStr());
			assert(is_src || is_dst);
			if (is_src)
				return 0;
			else if(is_dst)
				return 1;
			else
			{
				assert(0);
				return -1;
			}
		};

	auto OnTraverXmlNode = [this, I_Body] (const TiXmlNode* node) -> bool
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
						body_confs[I_Body(node)]->AddScale(b_name, x, y, z);
					}
				}
				else if("MotionPipe" == name)
				{
					const char* m_name[3][3] = {
						{"m11", "m12", "m13"},
						{"m21", "m22", "m23"},
						{"m31", "m32", "m33"},
					};

					bool all_entry_ij = true;
					for (int i_r = 0; i_r < 3 && all_entry_ij; i_r ++)
					{
						for (int i_c = 0; i_c < 3 && all_entry_ij; i_c ++)
						{
							all_entry_ij = (TIXML_SUCCESS == ele->QueryFloatAttribute(m_name[i_r][i_c], &m[i_r][i_c]));
						}
					}
					ret = all_entry_ij;
					IKAssert(all_entry_ij);
					if (!ret)
					{
						LOGIKVar(LogInfoFloat3x3_m, m);
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
						Pair.Add(j_from, j_to);
				}
				else if("EndEEF" == name)
				{
					const char* target_name = ele->Attribute("b_name");
					bool valid_target = (NULL != target_name);
					IKAssert(valid_target);
					ret = valid_target;
					if (valid_target)
						body_confs[I_Body(node)]->AddEEF(target_name);
				}
				else if((is_a_source = ("Source" == name))
					|| (is_a_desti = ("Destination" == name)))
				{
					const char* filename = ele->Attribute("file");
					body_confs[I_Body(node)]->SetFileName(filename);
				}
			}
			return ret;
		};

	return (TraverseBFS_XML_tree(doc, OnTraverXmlNode));
}