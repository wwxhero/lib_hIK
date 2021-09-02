#pragma once
#include <queue>
#include "tinyxml.h"
#include "MoNode.h"
#include "motion_pipeline.h"


namespace CONF
{
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

		void AllocCopyTo(B_Scale* dst) const
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

		void AllocCopyTo(const wchar_t* *a_name) const
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

	class CBodyConf
	{
	public:
		int Scale_alloc(B_Scale* &scales) const;
		static void Scale_free(B_Scale* scales, int n_scale);
		int EndEEF_alloc(const wchar_t ** &namesEEFs) const;
		static void EndEEF_free(const wchar_t** namesEEFs, int n_eefs);
		const wchar_t* file_w() const;
		const char* file_c() const;

		void AddScale(const char* name, Real x, Real y, Real z);
		void AddEEF(const char* name);
		void SetFileName(const char* fileName);

		BODY_TYPE type() const;


#ifdef _DEBUG
		void Dump_Dbg() const;
#endif

	public:
		std::vector<B_ScaleEx> m_scales;
		std::vector<Name> m_eefs;
		std::wstring m_fileName_w;
		std::string m_fileName_c;
	};

	class CPairsConf
	{
	public:
		void Add(const char* from, const char* to);
		int Data_alloc(int i_body, const wchar_t** &namesOnPair) const;
		static void Data_free(const wchar_t** namesOnPair, int n_pairs);
		int Data_alloc(const wchar_t* (**matches)[2]) const;
		static void Data_free(const wchar_t* (*matches)[2], int n_pairs);

#ifdef _DEBUG
		void Dump_Dbg() const;
#endif
	private:
		std::vector<std::pair<Name, Name>> m_pairs;
	};


	class CMotionPipeConf
	{
	public:

	private:
		CMotionPipeConf();
		~CMotionPipeConf();
		bool Initialize(TiXmlDocument* doc);

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

	public:
	#ifdef _DEBUG
		void Dump_Dbg() const;
	#endif
		static CMotionPipeConf* Load(const wchar_t* confXML);
		static void UnLoad(CMotionPipeConf* conf);
	public:
		CMoNode::TM_TYPE sync;
		Real m[3][3];
		CBodyConf Source, Destination;
		CPairsConf Pair;
	};

};
