#include "pch.h"
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

		IK_QSegment::Type type;
		const Real(*p_dex)[3] = NULL;
		if (name2confJoint.end() != it_conf_j)
		{
			type = it_conf_j->second->type;
			p_dex = &it_conf_j->second->dexterity;
		}
		else
		{
			type = IK_QSegment::R_xyz;
			p_dex = &dex_default;
		}
		IK_QSegment* seg = NULL;
		switch (type)
		{
			case IK_QSegment::R_xyz:
				seg = new IK_QIxyzSegment(*p_dex);
				break;
			case IK_QSegment::R_Spherical:
				seg = new IK_QSphericalSegment(*p_dex);
				break;
			default:
				break;
		}
		IKAssert(NULL != seg);
		if (seg->Initialize(seg_from[i_node], seg_to[i_node]))
			m_segments[n_segs ++] = seg;
	}
	m_segments.resize(n_segs);
	return n_segs > 0;
}