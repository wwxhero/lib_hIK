#pragma once
#include <map>
#include "ArtiBody.hpp"
#include "macro_helper.h"
#include "JointConf.hpp"

#define NumericalAlgor(algor)\
		(algor)&(CIKChain::NUM)

class CIKChain
{
public:
	enum Algor
	{
		None = 0x00000000,
		Proj = 0x00000001,
		DLS = 0x00000002,
		SDLS = 0x00000004,
		NUM = 0x00000006,
		Unknown,
	};

	static Real ERROR_MIN;

	// static const char* s_Algor_str[];
	// static Algor s_Algor_val[];
	// static Algor to_Algor(const char* algor_str);
	// static const char* from_Algor(Algor);
	DECLARE_ENUM_STR(Algor)
public:
	CIKChain(Algor algor, int n_iters);
	virtual ~CIKChain();
	virtual bool Init(const CArtiBodyNode* eef, int len, const std::vector<CONF::CJointConf>& joint_confs);
	virtual Real Error() const = 0;

	void SetupTarget(const std::map<std::wstring, CArtiBodyNode*>& nameSrc2bodyDst
					, const Eigen::Matrix3r& src2dst_w
					, const Eigen::Matrix3r& dst2src_w);

	virtual void Dump(std::ostream& info) const;

	virtual bool BeginUpdate(const Transform_TR& w2g);
	// this is a quick IK update solution
	virtual bool Update() = 0;
	virtual bool UpdateCompleted() const = 0;
	virtual void EndUpdate() {};

	void SetGRoot(CArtiBodyNode* root_g)
	{
		m_rootG = root_g;
	}

	int NIters() const
	{
		return m_nIters;
	}

	int NBodies() const
	{
		return (int)m_nodes.size();
	}

	bool operator < (const CIKChain& other)
	{
		int n_dist2root_this = 0;
		for (CArtiBodyNode* parent = m_rootG->GetParent()
			; NULL != parent
			; parent = parent->GetParent(), n_dist2root_this ++);

		int n_dist2root_other = 0;
		for (CArtiBodyNode* parent = other.m_rootG->GetParent()
			; NULL != parent
			; parent = parent->GetParent(), n_dist2root_other ++);

		return n_dist2root_this < n_dist2root_other
			|| (n_dist2root_this == n_dist2root_other && NBodies() < other.NBodies());
	}

	void Reset()
	{
		for (auto ik_node : m_nodes)
		{
			ik_node.joint->SetTranslation(Eigen::Vector3r::Zero());
			ik_node.joint->SetRotation(Eigen::Quaternionr::Identity());
		}
		IJoint* eef_j = m_eefSrc->GetJoint();
		eef_j->SetTranslation(Eigen::Vector3r::Zero());
		eef_j->SetRotation(Eigen::Quaternionr::Identity());
	}

public:
	const Algor c_algor;
	struct IKNode
	{
		CArtiBodyNode* body;
		IJoint* joint;
	};
protected:
	std::vector<IKNode> m_nodes;
	CArtiBodyNode* m_eefSrc;
	int m_nIters;
	CArtiBodyNode* m_rootG;
private:
	CArtiBodyNode* m_targetDst;
	Eigen::Matrix3r m_src2dstW_Offset;
	Eigen::Matrix3r m_dst2srcW;

};

class CIKChainNone : public CIKChain
{
public:
	CIKChainNone();
	virtual ~CIKChainNone();
	virtual Real Error() const
	{
		return (Real)0;
	}

	virtual void Dump(std::ostream& info) const override;
	// this is a quick IK update solution
	virtual bool Update();
	virtual bool UpdateCompleted() const
	{
		return true;
	}
};

class CIKChainProj : public CIKChain
{
public:
	CIKChainProj(const Real norm[3]);
	virtual ~CIKChainProj();
	virtual bool Init(const CArtiBodyNode* eef, int len, const std::vector<CONF::CJointConf>&) override;

	virtual Real Error() const
	{
		return (Real)0;
	}

	virtual void Dump(std::ostream& info) const override;
	virtual bool BeginUpdate(const Transform_TR& w2g) override;
	// this is a quick IK update solution
	virtual bool Update();
	virtual bool UpdateCompleted() const
	{
		return true;
	}
private:
	Plane m_terrainW;
	Plane m_terrainG;
};

