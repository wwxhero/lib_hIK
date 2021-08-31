#pragma once
#include <queue>
#include "tinyxml.h"
#include "conf_mopipe.h"


class CBodyConf
{
public:
	int Scale(B_Scale* &scales) const;
	int EndEEF(const wchar_t ** &namesEEFs) const;
	const wchar_t* file() const;

	void AddScale(const char* name, Real x, Real y, Real z);
	void AddEEF(const char* name);
	void SetFileName(const char* fileName);
public:

};

class CPairsConf
{
public:
	void Add(const char* from, const char* to);
	int Data(int i_body, const wchar_t** &namesOnPair) const;
	int Data(const wchar_t* (**matches)[2]) const;

};


class CMotionPipeConf
{
public:
	enum T_Sync {cross, homo, unknown};
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
	T_Sync sync;
	Real m[3][3];
	CBodyConf Source, Destination;
	CPairsConf Pair;
};
