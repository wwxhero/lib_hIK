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
		Proj = 0x00000001
		, DLS = 0x00000002
		, SDLS = 0x00000004
		, NUM = 0x00000006
		, Unknown
	};

	// static const char* s_Algor_str[];
	// static Algor s_Algor_val[];
	// static Algor to_Algor(const char* algor_str);
	// static const char* from_Algor(Algor);
	DECLARE_ENUM_STR(Algor)
public:
	CIKChain(Algor algor, int n_iters);
	virtual ~CIKChain();
	virtual bool Init(const CArtiBodyNode* eef, int len, const std::vector<CONF::CJointConf>& joint_confs);
	void SetupTarget(const std::map<std::wstring, CArtiBodyNode*>& nameSrc2bodyDst
					, const Eigen::Matrix3r& src2dst_w
					, const Eigen::Matrix3r& dst2src_w);

	virtual void Dump(std::stringstream& info) const;

	void BeginUpdate();

	virtual void UpdateNext(int step)
	{
	}

	// this is a quick IK update solution
	virtual void UpdateAll()
	{

	}

	void EndUpdate()
	{

	}

	int NIters() const
	{
		return m_nIters;
	}

	int NBodies() const
	{
		return (int)m_nodes.size();
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
private:
	CArtiBodyNode* m_targetDst;
	Eigen::Matrix3r m_src2dstW_Offset;
	Eigen::Matrix3r m_dst2srcW;

};

class CIKChainProj : public CIKChain
{
public:
	CIKChainProj(const Real norm[3]);
	virtual bool Init(const CArtiBodyNode* eef, int len, const std::vector<CONF::CJointConf>&) override;
	virtual void Dump(std::stringstream& info) const override;
	virtual void UpdateNext(int step) override;

	// this is a quick IK update solution
	virtual void UpdateAll() override;
private:
	void Update();
	Plane m_terrain;
};

