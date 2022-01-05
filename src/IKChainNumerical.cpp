#include "pch.h"
#include "Math.hpp"
#include "IKChainNumerical.hpp"

bool CIKChainNumerical::Init(const CArtiBodyNode* eef, int len, const std::vector<CONF::CJointConf>& joint_confs)
{
	if (!Super::Init(eef, len, joint_confs))
		return false;
	for (auto seg : m_segments)
		delete seg;
	m_segments.clear();

	std::map<std::string, const CONF::CJointConf*> name2confJoint;
	for (auto& confJoint : joint_confs)
		name2confJoint[confJoint.name] = &confJoint;

	int n_nodes = (int)m_nodes.size();

	std::vector<CArtiBodyNode*> seg_from(n_nodes);
	std::vector<CArtiBodyNode*> seg_to(n_nodes);
	int i_node = 0;
	seg_from[i_node] = m_nodes[i_node].body;
	int i_node_m = 0;
	for (i_node = 1; i_node < n_nodes; i_node ++, i_node_m ++)
	{
		auto& node_i = m_nodes[i_node];
		seg_to[i_node_m] = node_i.body;
		seg_from[i_node] = node_i.body;
	}
	seg_to[i_node_m] = const_cast<CArtiBodyNode*>(eef);

	m_segments.resize(n_nodes);
	int n_segs = 0;
	for (i_node = 0; i_node < n_nodes; i_node ++)
	{
		auto it_conf_j = name2confJoint.find(seg_from[i_node]->GetName_c());
		const auto type_default = IK_QSegment::R_xyz;
		const Real dex_default[3] = { (Real)1, (Real)1, (Real)1 };
		const Real lim_default[3][2] = {
											  {IK_QSegment::MIN_THETA-1, IK_QSegment::MAX_THETA+1}
											, {IK_QSegment::MIN_TAU-1, IK_QSegment::MAX_TAU+1}
											, {IK_QSegment::MIN_PHI-1, IK_QSegment::MAX_PHI+1}
										};

		IK_QSegment::Type type;
		const Real (*p_dex)[3] = NULL;
		const Real (*p_lim)[3][2] = NULL;
		if (name2confJoint.end() != it_conf_j)
		{
			type = it_conf_j->second->type;
			p_dex = &it_conf_j->second->dexterity;
			p_lim = &it_conf_j->second->lim;
		}
		else
		{
			type = IK_QSegment::R_xyz;
			p_dex = &dex_default;
			p_lim = &lim_default;
		}
		int n_dofs = 0;

		IK_QSegment* seg = NULL;
		switch (type)
		{
			case IK_QSegment::R_xyz:
				seg = new IK_QIxyzSegment();
				n_dofs = 3;
				break;
			case IK_QSegment::R_Spherical:
				seg = new IK_QSphericalSegment();
				n_dofs = 3;
				break;
			default:
				break;
		}
		IKAssert(NULL != seg);
		for (int i_dof = 0; i_dof < n_dofs; i_dof ++)
		{
			seg->SetWeight(i_dof, (*p_dex)[i_dof]);
			seg->SetLimit((IK_QSegment::DOFLim)i_dof, (*p_lim)[i_dof]);
		}
		if (seg->Initialize(seg_from[i_node], seg_to[i_node]))
			m_segments[n_segs ++] = seg;
	}
	m_segments.resize(n_segs);
	return n_segs > 0;
}

bool CIKChainNumerical::Update()
{
	// // boost numerical computation with posture graph
	// CPGSearch* search = StartSearch(m_PG);
	// Transform* t_kp = m_eef->GetGoal();

	// HANDLE threads_h[n_threads] = {H_INVALID};
	// auto Initiate_Update_A = [auto& pg = m_PG
	// 						, auto& theta = m_theta
	// 						, auto& rootG_s = m_rootG
	// 						, auto& storage_t = m_storageT](CPGSearch* search, const Transform* t_kp, int i_thread) -> HANDLE
	// 	{
	// 		pg.LocateMinimum(search, t_kp)
	// 		auto& rootG_i = storage_t[i_thread].m_rootG;
	// 		rootG_s->Serialize<true>(theta);
	// 		rootG_i->Serialize<false>(theta);
	// 		return CreateThreadFunc(Update_AnyThread(i_thread));
	// 	};

	// for (int i_thread = 0; i_thread < n_threads; i_thread ++)
	// {
	// 	threads_h[i_thread] = Initiate_Update_A(search, t_kp, i_thread);
	// }

	// bool solved = false;

	// while(!solved &&
	// 	int i_thread = ::WaitForMultipleObjects(m_threads, n_threads))
	// {
	// 	solved = m_threads[i_thread].validRes;
	// 	if (!solved)
	// 	{
	// 		threads_h[i_thread] = Initiate_Update_A(search, t_kp, i_thread);
	// 	}
	// 	else
	// 	{
	// 		auto& rootG_i = m_threads[i_thread].m_rootG;
	// 		rootG_i->Serialize<true>(m_theta);
	// 		m_rootG->Serialize<false>(m_theta);
	// 		::KillThreads(threads_h);
	// 	}
	// }

	// StopSearch(search, m_pg);

	return Update_AnyThread();
}

Real CIKChainNumerical::Error() const
{
	_TRANSFORM tm_t;
	m_eefSrc->GetGoal(tm_t);
	const Transform* tm_eef = m_eefSrc->GetTransformLocal2World();
	Real err_r = ::Error_q(Eigen::Quaternionr(tm_t.r.w, tm_t.r.x, tm_t.r.y, tm_t.r.z)
						, Transform::getRotation_q(tm_eef));
	// the following error is computed based on CCD IK
	Real err_tt = 0;
	Eigen::Vector3r tt_t(tm_t.tt.x, tm_t.tt.y, tm_t.tt.z);
	Eigen::Vector3r tt_eef = tm_eef->getTranslation();
	auto it_seg = m_segments.end();
	const Real half = (Real)0.5;
	Real sigma_stiffness = (Real)0;
	while (it_seg != m_segments.begin())
	{
		it_seg --;
		auto seg = *it_seg;
		Eigen::Vector3r p_i = seg->GlobalStart();
		Eigen::Vector3r pi_t = tt_t - p_i;
		Eigen::Vector3r pi_eef = tt_eef - p_i;
		Real norm_pi_t = pi_t.norm();
		Real norm_pi_eef = pi_eef.norm();
		if (norm_pi_t > c_epsilonsqrt
			&& norm_pi_eef > c_epsilonsqrt)
		{
			Real err_pi = half * ((Real)1 - pi_eef.dot(pi_t)/(norm_pi_t * norm_pi_eef));
			Real stiffness_i = seg->Stiffness();
			err_tt += err_pi * stiffness_i;
			sigma_stiffness = sigma_stiffness + stiffness_i;
		}
	}

	err_tt = err_tt / sigma_stiffness;

	LOGIKVar(LogInfoReal, err_tt);
	LOGIKVar(LogInfoReal, err_r);

	return c_taskW_r * err_r
		+ c_taskW_t * err_tt;
}