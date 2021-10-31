#pragma once
#include "pch.h"
#include <queue>
#include "tinyxml.h"
#include "MoNode.hpp"
#include "motion_pipeline.h"
#include "JointConf.hpp"
#include "IKChain.hpp"

namespace CONF
{
	class B_ScaleEx : public B_Scale
	{
	public:
		B_ScaleEx(const char* name, Real x, Real y, Real z)
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

		operator std::wstring() const
		{
			return m_str;
		}
	private:
		std::wstring m_str;
	};

	template<class TDerived>
	class ConfDoc
	{
	public:
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

		static TDerived* Load(const std::wstring& confXML)
		{
			std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
			std::string path_c = converter.to_bytes(confXML);
			return Load(path_c);
		}

		static TDerived* Load(const std::string& path_c)
		{
			TiXmlDocument* doc = new TiXmlDocument(path_c);
			TDerived* conf = NULL;
			bool loadOkay = doc->LoadFile();
			if (loadOkay)
			{
				conf = new TDerived();
				if (!conf->Initialize(doc))
				{
					delete conf;
					conf = NULL;
				}
			}
			delete doc;
			return conf;
		}

		static void UnLoad(TDerived* conf)
		{
			delete conf;
		}

		virtual bool Initialize(const TiXmlNode* doc) = 0;
	};

	class CIKChainConf
	{
	public:
		CIKChainConf(const char* eef_name
					, int len
					, CIKChain::Algor algor
					, Real weight_p
					, Real weight_r
					, int n_iter
					, const char* P_Graph);
		CIKChainConf(const char* eef_name
					, int len
					, CIKChain::Algor algor
					, Real up[3]);
		CIKChainConf(const CIKChainConf& src);
		~CIKChainConf();
		void AddJoint(const char* attri_values[5]);
#ifdef _DEBUG
		void Dump_Dbg() const;
#endif
	public:
		std::string eef;
		int len;
		CIKChain::Algor algor;
		union
		{
			struct // Numerical
			{
				Real weight_p;
				Real weight_r;
				int n_iter;
			};
			struct //Proj
			{
				Real up[3];
			};
		};
		std::string P_Graph;
		std::vector<CJointConf> Joints;
	};

	class CBodyConf : public ConfDoc<CBodyConf>
	{
		friend class ConfDoc<CBodyConf>;
	public:
		CBodyConf();
		int Scale_alloc(B_Scale* &scales) const;
		static void Scale_free(B_Scale* scales, int n_scale);
		int Targets_alloc(const wchar_t ** &nameTargets) const;
		static void Targets_free(const wchar_t** nameTargets, int n_Targets);
		const wchar_t* file_w() const;
		const char* file_c() const;
		const wchar_t* record_w() const;
		const char* record_c() const;
		const wchar_t* PG_dir_w() const;
		const char* PG_dir_c() const;

		void AddScale(const char* name, Real x, Real y, Real z);
		void AddTarget(const char* name);
		void SetFileName(const char* fileName);
		void SetRecord(const char* rc_file);
		void SetPGDir(const char* pg_dir);

		BODY_TYPE type() const;

		void AddIKChain(const char* eef_name
						, int len
						, CIKChain::Algor algor
						, Real weight_p
						, Real weight_r
						, int n_iter
						, const char* P_Graph);
		void AddIKChain(const char* eef_name
						, int len
						, CIKChain::Algor algor
						, Real up[3]);
		CIKChainConf* GetIKChain(const char* eef_name);
		int Name2IKChainIdx(const char* eef_name) const;
#ifdef _DEBUG
		void Dump_Dbg() const;
#endif
		virtual bool Initialize(const TiXmlNode* doc);
	private:
		std::vector<B_ScaleEx> m_scales;
		std::vector<Name> m_targets;
		std::wstring m_fileName_w;
		std::string m_fileName_c;
		std::wstring m_record_w;
		std::string m_record_c;
		std::wstring m_pgDir_w;
		std::string m_pgDir_c;
	public:
		std::vector<CIKChainConf> IK_Chains;
	private:
		std::map<std::string, int> m_name2chainIdx;
	};

	class CPairsConf
	{
	public:
		void Add(const char* from, const char* to);
		int Data_alloc(int i_body, const wchar_t** &namesOnPair) const;
		static void Data_free(const wchar_t** namesOnPair, int n_pairs);
		int Data_alloc(const wchar_t* (**matches)[2]) const;
		static void Data_free(const wchar_t* (*matches)[2], int n_pairs);
		void Map(std::map<std::wstring, std::wstring>& name2name, bool forward);
#ifdef _DEBUG
		void Dump_Dbg() const;
#endif
	private:
		std::vector<std::pair<Name, Name>> m_pairs;
	};


	class CMotionPipeConf : public ConfDoc<CMotionPipeConf>
	{
		friend class ConfDoc<CMotionPipeConf>;
	private:
		CMotionPipeConf();
		~CMotionPipeConf();
		virtual bool Initialize(const TiXmlNode* doc);

	public:
	#ifdef _DEBUG
		void Dump_Dbg() const;
	#endif
	public:
		CMoNode::RETAR_TYPE sync;
		Real m[3][3];
		Real m_inv[3][3];
		CBodyConf Source, Destination;
		CPairsConf Pair;
	};

};
