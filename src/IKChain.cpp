#include "pch.h"
#include "IKChain.hpp"

BEGIN_ENUM_STR(CIKChain, Algor)
	ENUM_ITEM(Proj)
	ENUM_ITEM(DLS)
	ENUM_ITEM(SDLS)
	ENUM_ITEM(Unknown)
END_ENUM_STR(CIKChain, Algor)

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CIKChain:

CIKChain::CIKChain(Algor algor, int n_iters)
	: c_algor(algor)
	, m_eefSrc(NULL)
	, m_rootG(NULL)
	, m_targetDst(NULL)
	, m_nIters(n_iters)
{
}

CIKChain::~CIKChain()
{
}

bool CIKChain::Init(const CArtiBodyNode* eef, int len, const std::vector<CONF::CJointConf>&)
{
	IKAssert(Proj != c_algor || 1 == len); // (c_algor == Proj) -> 1 == len
	m_eefSrc = const_cast<CArtiBodyNode*>(eef);
	m_nodes.resize(len);
	CArtiBodyNode* body_p = NULL;
	int i_start = len - 1;
	int i_end = -1;
	int i;
	for ( i = i_start, body_p = m_eefSrc->GetParent()
		; i > i_end && NULL != body_p
		; i --, body_p = body_p->GetParent())
	{
		IKNode& node_i = m_nodes[i];
		node_i.joint = body_p->GetJoint();
		node_i.body = body_p;
	}

	bool initialized = (i == i_end && len > 0);
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

bool CIKChain::BeginUpdate(const Transform_TR& w2g)
{
	IKAssert(NULL != m_eefSrc
		&& NULL != m_targetDst);
	_TRANSFORM tm;
	m_targetDst->GetGoal(tm);
	Transform_TRS target_dst_w(tm);

	Transform_TRS target_src_w;
	Eigen::Matrix3r target_linear_src_w = m_dst2srcW * target_dst_w.getLinear() * m_src2dstW_Offset;
	Eigen::Vector3r target_tt_src_w = m_dst2srcW * target_dst_w.getTranslation();
	target_src_w.linear() = target_linear_src_w;
	target_src_w.translation() = target_tt_src_w;

	_TRANSFORM target_src_w_tm;
	target_src_w.CopyTo(target_src_w_tm);
	const Real err_b = (Real)0.05;
	const Real sd = (Real)1;
	const Real range[] = { sd - err_b, sd + err_b };
	IKAssert(range[0] < target_src_w_tm.s.x && target_src_w_tm.s.x < range[1]
		&&	range[0] < target_src_w_tm.s.y && target_src_w_tm.s.y < range[1]
		&&	range[0] < target_src_w_tm.s.z && target_src_w_tm.s.z < range[1]);
	target_src_w_tm.s.x = target_src_w_tm.s.y = target_src_w_tm.s.z = (Real)1;

	_TRANSFORM eef_src_w_tm;
	m_eefSrc->GetTransformLocal2World()->CopyTo(eef_src_w_tm);
	bool valid_update = !Equal(eef_src_w_tm, target_src_w_tm);
	if (valid_update)
	{
		Transform_TR target_src_g;
		Transform_TR target_src_w_tr(target_src_w_tm);
		target_src_g.Update(w2g, target_src_w_tr);
		_TRANSFORM target_src_g_tm;
		target_src_g.CopyTo(target_src_g_tm);
		m_eefSrc->UpdateGoal(target_src_g_tm);
	}

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
	LOGIKVar(LogInfoBool, valid_update);
	LOGIKFlush();
#endif
	return valid_update;
}

void CIKChain::Dump(std::stringstream& info) const
{
	info << "{";
	for (auto seg : m_nodes)
		info <<" "<< seg.body->GetName_c();
	info << " " << m_eefSrc->GetName_c();
	info << "}";
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CIKChainProj:

CIKChainProj::CIKChainProj(const Real norm[3])
	: CIKChain(Proj, 1)
{
	m_terrainW.n << norm[0], norm[1], norm[2];
	IKAssert(UnitVec(m_terrainW.n));
}

CIKChainProj::~CIKChainProj()
{
}

bool CIKChainProj::Init(const CArtiBodyNode* eef, int len, const std::vector<CONF::CJointConf>& joints_conf)
{
	bool initialized = (1 == len)
					&&  CIKChain::Init(eef, len, joints_conf);
	if (initialized)
	{
		const Transform* l2w_0 = m_nodes[0].body->GetTransformLocal2World();
		m_terrainW.p = l2w_0->getTranslation();
	}
	return initialized;
}

void CIKChainProj::Dump(std::stringstream& info) const
{
	info << "CIKChainProj:";
	CIKChain::Dump(info);
}

bool CIKChainProj::BeginUpdate(const Transform_TR& w2g)
{
	if (!CIKChain::BeginUpdate(w2g))
		return false;
	m_terrainG = w2g.Apply(m_terrainW);
	return true;
}

bool CIKChainProj::UpdateAll()
{
	_TRANSFORM goal;
	m_eefSrc->GetGoal(goal);
	IKAssert(NoScale(goal));

	Eigen::Quaternionr R(goal.r.w, goal.r.x, goal.r.y, goal.r.z);
	Eigen::Vector3r T(goal.tt.x, goal.tt.y, goal.tt.z);

	Eigen::Quaternionr R_0 = Eigen::Quaternionr::Identity();
	Eigen::Quaternionr R_1 = R;

	Eigen::Vector3r T_0 = m_terrainG.ProjP(T);
	Eigen::Vector3r T_1 = T - T_0;

	Eigen::Quaternionr* Rs[] = {&R_0, &R_1};
	Eigen::Vector3r* Ts[] = {&T_0, &T_1};

	CArtiBodyNode* bodies[] = {m_nodes[0].body, m_eefSrc};
	IJoint* joints[] = {m_nodes[0].joint, m_eefSrc->GetJoint()};

	for (int i_kina = 0; i_kina < 2; i_kina ++)
	{
		Transform_TR l2p_i;
		l2p_i.setRotation(*Rs[i_kina]);
		l2p_i.setTranslation(*Ts[i_kina]);
		const Transform* l2p_i_0_temp = bodies[i_kina]->GetTransformLocal2Parent0();
		IKAssert(t_tr == l2p_i_0_temp->Type());
		const Transform_TR* l2p_i_0 = static_cast<const Transform_TR*>(l2p_i_0_temp);
		// l2p_0_0 * delta = l2p_0;
		//		=> delta = (l2p_0_0^-1)*l2p_0
		Transform_TR delta_i;
		delta_i.Update(l2p_i_0->inverse(), l2p_i);
		auto joint_i = joints[i_kina];
		joint_i->SetRotation(delta_i.getRotation_q());
		joint_i->SetTranslation(delta_i.getTranslation());
	}

	// CArtiBodyTree::FK_Update<true>(m_nodes[0].body);
	return true;
}