#include "pch.h"
#include "IKChain.hpp"
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CIKChain:

CIKChain::CIKChain()
	: m_eefSrc(NULL)
	, m_targetDst(NULL)
	, m_nSteps(20)
{
}

bool CIKChain::Init(const CArtiBodyNode* eef, int len)
{
	m_eefSrc = const_cast<CArtiBodyNode*>(eef);
	m_bodies.resize(len);
	CArtiBodyNode* body_p = NULL;
	int i_start = len - 1;
	int i_end = -1;
	int i;
	for ( i = i_start, body_p = m_eefSrc->GetParent()
		; i > i_end && NULL != body_p
		; i --, body_p = body_p->GetParent())
	{
		m_bodies[i] = body_p->GetJoint();
	}

	bool initialized = (i == i_end);
	IKAssert(initialized);
	return initialized;
}

void CIKChain::SetupTarget(const std::map<std::wstring, CArtiBodyNode*>& nameSrc2bodyDst
						, const Eigen::Matrix3r& src2dst_w
						, const Eigen::Matrix3r& dst2src_w)
{
	IKAssert(NULL != m_eefSrc);
	auto it_target = nameSrc2bodyDst.find(m_eefSrc->GetName_w());
	IKAssert(nameSrc2bodyDst.end() != it_target);
	m_targetDst = it_target->second;
	m_dst2srcW = dst2src_w;

	const Transform* local2world_dst = m_targetDst->GetTransformLocal2World();
	Eigen::Matrix3r linear_local2world_dst = local2world_dst->getLinear();
	const Transform* local2world_src = m_eefSrc->GetTransformLocal2World();
	Eigen::Matrix3r linear_local2world_src = local2world_src->getLinear();
	Eigen::Matrix3r linear_local2world_prime_dst = src2dst_w * linear_local2world_src * m_dst2srcW;
	Eigen::Matrix3r offsetDst = linear_local2world_dst.inverse() * linear_local2world_prime_dst;
	m_src2dstW_Offset = offsetDst * src2dst_w;
}

void CIKChain::BeginUpdate()
{
	IKAssert(NULL != m_eefSrc
		&& NULL != m_targetDst);
	Transform_TRS target_dst_w;
	m_targetDst->GetGoal(target_dst_w);

	Transform_TRS target_src_w;
	Eigen::Matrix3r target_linear_src_w = m_dst2srcW * target_dst_w.getLinear() * m_src2dstW_Offset;
	Eigen::Vector3r target_tt_src_w = m_dst2srcW * target_dst_w.getTranslation();
	target_src_w.linear() = target_linear_src_w;
	target_src_w.translation() = target_tt_src_w;
	m_eefSrc->SetGoal(target_src_w);

#ifdef _DEBUG
	const Transform* eef_src_w = m_eefSrc->GetTransformLocal2World();
	Eigen::Vector3r offset_T = eef_src_w->getTranslation() - target_src_w.getTranslation();
	std::stringstream logInfo;
	logInfo << "reaching from " << m_eefSrc->GetName_c()
		<< " to " << m_targetDst->GetName_c()
		<< " at\n\ttarget_src = [" << target_src_w.ToString().c_str() << "]"
		<<    "\n\t   eef_src = [" << eef_src_w->ToString().c_str() << "]"
		<< "\n\tOffset_T = \n[" << offset_T << "]";
	LOGIK(logInfo.str().c_str());
	LOGIKFlush();
#endif
}

void CIKChain::Dump(std::stringstream& info) const
{
	info << "{";
	for (auto body : m_bodies)
		info <<" "<< body->GetName_c();
	info << " " << m_eefSrc->GetName_c();
	info << "}";
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CIKChainProj:

CIKChainProj::CIKChainProj()
{

}