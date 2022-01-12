#pragma once
#include <ostream>

#include "MotionPipeConf.hpp"
#include "PGRuntimeParallel.hpp"
#include "IKGroup.hpp"

class CIKGroupNode : public TreeNode<CIKGroupNode>
{
public:
	CIKGroupNode(CArtiBodyNode* root);
	explicit CIKGroupNode(CIKGroupNode& src);
	~CIKGroupNode();

	void Join(CIKChain* chain)
	{
		m_primary.Join(chain);
		m_secondary.Joint(chain);
	}

	void IKUpdate();
	void IKReset();

	void SetupTargets(const std::map<std::wstring, CArtiBodyNode*>& nameSrc2bodyDst, const Eigen::Matrix3r& src2dst_w, const Eigen::Matrix3r& dst2src_w);
	void LoadPostureGraph(const char* pgDir, int radius);
	virtual void Dump(int indent) const override;
	void Dump(int indent, std::ostream& out) const;

	bool Empty() const
	{
		return m_primary.Empty();
	}

	CArtiBodyNode* RootBody() const
	{
		return m_primary.RootBody();
	}
protected:
	CIKGroup m_primary;
	CIKGroupsParallel m_secondary;
	CPGRuntimeParallel* m_pg;
};

class CIKGroupTree : public Tree<CIKGroupNode>
{
public:
	static CIKGroupNode* Generate(const CArtiBodyNode* root, const CONF::CBodyConf& ikChainConf);
	static void SetupTargets(CIKGroupNode* root_ik, const std::map<std::wstring, CArtiBodyNode*>& nameSrc2bodyDst, const Eigen::Matrix3r& src2dst_w, const Eigen::Matrix3r& dst2src_w);
	static void LoadPG(CIKGroupNode* root_ik, const char* dirPath, int radius);
};


