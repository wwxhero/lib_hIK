#pragma once
#include <map>
#include "ArtiBody.h"
#include "macro_helper.h"

#define NumericalAlgor(algor)\
		(algor)&(CIKChain::NUM)

class CIKChain
{
public:
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
	// static Algor toAlgor(const char* algor_str);
	DECLARE_ENUM_STR(Algor)



public:
	CIKChain();
	bool Init(const CArtiBodyNode* eef, int len);
	void SetupTarget(const std::map<std::wstring, CArtiBodyNode*>& nameSrc2bodyDst
					, const Eigen::Matrix3r& src2dst_w
					, const Eigen::Matrix3r& dst2src_w);

	void Dump(std::stringstream& info) const;
	virtual void BeginUpdate();

	virtual void UpdateNext(int step)
	{
	}

	// this is a quick IK update solution
	virtual void UpdateAll()
	{

	}

	virtual void EndUpdate()
	{

	}

	int NSteps() const
	{
		return m_nSteps;
	}

	int NBodies() const
	{
		return (int)m_bodies.size();
	}
private:
	std::vector<IJoint*> m_bodies;
	CArtiBodyNode* m_eefSrc;
	CArtiBodyNode* m_targetDst;
	int m_nSteps;
	Eigen::Matrix3r m_src2dstW_Offset;
	Eigen::Matrix3r m_dst2srcW;
};

class CIKChainProj : public CIKChain
{
public:
	CIKChainProj();
};