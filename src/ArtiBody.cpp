#include "pch.h"
#include <queue>
#include <stack>
#include <map>
#include <set>
#include <sstream>
#include "articulated_body.h"
#include "ArtiBody.hpp"
#include "ik_logger.h"
#include "handle_helper.hpp"



CArtiBodyNode* CArtiBodyTree::CreateAnimNode(const wchar_t* name, const _TRANSFORM* tm)
{
	return CreateAnimNodeInternal(name, tm);
}

CArtiBodyNode* CArtiBodyTree::CreateAnimNode(const char* name, const _TRANSFORM* tm)
{
	return CreateAnimNodeInternal(name, tm);
}

CArtiBodyNode* CArtiBodyTree::CreateSimNode(const wchar_t* name, const _TRANSFORM* tm, BODY_TYPE type, TM_TYPE jtm, bool local)
{
	return CreateSimNodeInternal(name, tm, type, jtm, local);
}

CArtiBodyNode* CArtiBodyTree::CreateSimNode(const char* name, const _TRANSFORM* tm, BODY_TYPE type, TM_TYPE jtm, bool local)
{
	return CreateSimNodeInternal(name, tm, type, jtm, local);
}

bool CArtiBodyTree::CloneNode_fbx(const CArtiBodyNode* src, CArtiBodyNode** dst, const wchar_t* name_dst_opt)
{
	*dst = NULL;
	const Transform* l2p_tm = src->GetTransformLocal2Parent();
	_TRANSFORM l2p_0_tm;
	l2p_tm->CopyTo(l2p_0_tm);

	CArtiBodyNode* src_parent = src->GetParent();
	bool is_root = (NULL == src_parent);
	if (is_root)
		l2p_0_tm.tt.x = l2p_0_tm.tt.y = l2p_0_tm.tt.z = 0;

	const wchar_t* name_dst = (NULL == name_dst_opt ? src->GetName_w() : name_dst_opt);

	*dst = CreateAnimNode(name_dst, &l2p_0_tm);
	return NULL != *dst;
}

bool CArtiBodyTree::CloneNode_bvh(const CArtiBodyNode* src, CArtiBodyNode** dst, const wchar_t* name_dst_opt)
{
	*dst = NULL;
	CArtiBodyNode* src_parent = src->GetParent();
	bool jtm_copy = (NULL == name_dst_opt);
	const wchar_t* name_dst = (NULL == name_dst_opt ? src->GetName_w() : name_dst_opt);
	TM_TYPE jtm = TM_TYPE::t_none;
	bool is_root = (NULL == src_parent);
	if (jtm_copy)
		jtm = src->c_jtmflag;
	else
	{
		if (is_root)
			jtm = t_tr;
		else
			jtm = t_r;
	}

	const Transform* l2w_this = src->GetTransformLocal2World();
	Eigen::Vector3r offset;
	if (is_root)
	{
		offset = l2w_this->getTranslation();
	}
	else
	{
		const Transform* l2w_parent = src_parent->GetTransformLocal2World();
		const Transform* l2w_this = src->GetTransformLocal2World();
		offset = l2w_this->getTranslation() - l2w_parent->getTranslation();
	}

	_TRANSFORM l2p_0_tm = {
		{1, 1, 1},
		{1, 0, 0, 0},
		{offset.x(), offset.y(), offset.z()}
	};
	*dst = CreateSimNode(name_dst, &l2p_0_tm, bvh, jtm, true);
	return NULL != *dst;
}



bool CArtiBodyTree::CloneNode_htr(const CArtiBodyNode* src, CArtiBodyNode** dst, const Eigen::Matrix3r& src2dst_w, const wchar_t* name_dst_opt)
{
	// if (0 == strcmp(src->GetName_c(), "LeftHand"))
	//  	DebugBreak(); //a zero length bone
	// if (0 == strcmp(src->GetName_c(), "LeftForeArm"))
	//  	DebugBreak(); //a zero length bone
	const CArtiBodyNode* child_src = src->GetFirstChild();
	const CArtiBodyNode* parent_src = src->GetParent();
	bool is_root = (NULL == parent_src);
	bool is_leaf = (NULL == child_src);
	bool single_node = (is_leaf && is_root);
	*dst = NULL;
	if (single_node)
		return CloneNode_bvh(src, dst, name_dst_opt);
	else // (!single_node)
	{
		bool valid_nor = false;
		Eigen::Vector3r nor = Eigen::Vector3r::Zero();

		Eigen::Vector3r tt_l2p;
		const Eigen::Vector3r& tt_this = src2dst_w * src->GetTransformLocal2World()->getTranslation();

		auto GetDir = [](const Eigen::Vector3r &tt_from, const Eigen::Vector3r &tt_to, Eigen::Vector3r& dir) -> bool
			{
				auto vec = tt_to - tt_from;
				auto abs_vec = vec.norm();
				bool valid_dir = (abs_vec > 0.01); //no neighboring joints are closer than 1 cm
				if (valid_dir)
				{
					auto abs_vec_inv = 1 / abs_vec;
					dir.x() = vec.x() * abs_vec_inv;
					dir.y() = vec.y() * abs_vec_inv;
					dir.z() = vec.z() * abs_vec_inv;
				}
				return valid_dir;
			};

		auto onSearchBody = [&nor, tt_this, GetDir, &src2dst_w](const CArtiBodyNode* node_src) -> bool
			{
				Eigen::Vector3r tt_other = src2dst_w * node_src->GetTransformLocal2World()->getTranslation();
				bool valid_dir = GetDir(tt_this, tt_other, nor);
				return valid_dir;
			};

		if (!(valid_nor = Tree<CArtiBodyNode>::SearchBFS(src, onSearchBody)))
		{
			while (!valid_nor
				&& (NULL != parent_src))
			{
				Eigen::Vector3r tt_other = src2dst_w * parent_src->GetTransformLocal2World()->getTranslation();
				valid_nor = GetDir(tt_other, tt_this, nor);
				parent_src = parent_src->GetParent();
			}
		}

		Eigen::Quaternionr rot_q;
		if (valid_nor)
		{
			Eigen::Matrix3r rot_m;
			vec_roll_to_mat3_normalized(nor, 0, rot_m);
			rot_q = rot_m;
#ifdef _DEBUG
			Eigen::Vector3r nor_prime = rot_m * Eigen::Vector3r::UnitY();
			Eigen::Vector3r err_v = nor-nor_prime;
			Real err_norm = err_v.norm();
			bool matrix_corr = (err_norm < c_epsilon);
			IKAssert(matrix_corr);
			if (!matrix_corr)
			{
				std::stringstream msg;
				msg << src->GetName_c() << ":" << std::endl
										 << "\tnor = \n" << nor << std::endl
										 << "\trot_m = \n"<< rot_m << std::endl;
				LOGIK(msg.str().c_str());
			}
#endif
		}
		else
			rot_q = Eigen::Quaternionr::Identity();

		_TRANSFORM tm = {
			{1, 1, 1},
			{rot_q.w(), rot_q.x(), rot_q.y(), rot_q.z()},
			{tt_this.x(), tt_this.y(), tt_this.z()}
		};
		bool jtm_copy = (NULL == name_dst_opt);
		TM_TYPE tm_type = t_none;
		if (jtm_copy)
			tm_type = src->c_jtmflag;
		else
			tm_type = ((is_root || NULL == parent_src->GetParent()) ? t_tr : t_r); 		// entity node and hip node are tr nodes: translation + rotation
		const wchar_t* name_dst = (NULL == name_dst_opt ? src->GetName_w() : name_dst_opt);
		*dst = CreateSimNode(name_dst, &tm, htr, tm_type, false);

		return NULL != *dst;
	}

}

bool CArtiBodyTree::Clone_htr(const CArtiBodyNode* src, CArtiBodyNode** dst, const wchar_t* (*a_matches)[2], int n_matches, bool src_on_match0, const Eigen::Matrix3r& src2dst_w)
{
	if (NULL == src)
	{
		*dst = NULL;
		return false;
	}

	std::map<std::wstring, std::wstring> matches;

	int i_src = src_on_match0 ? 0 : 1;
	int i_dst = (!i_src);
	for (int i_match = 0; i_match < n_matches; i_match++)
	{
		std::wstring name_src(a_matches[i_match][i_src]);
		std::wstring name_dst(a_matches[i_match][i_dst]);
		matches[name_src] = name_dst;
	}
	std::map<std::wstring, std::wstring>& c_matches = matches;

	auto CloneNode = [&c_matches, &src2dst_w] (const CArtiBodyNode* node_src, CArtiBodyNode** node_dst) -> bool
				{
					auto it = c_matches.find(node_src->GetName_w());
					bool interest = (it != c_matches.end());
					*node_dst = NULL;
					if ( interest )
					{
						bool cloned = CArtiBodyTree::CloneNode_htr(node_src, node_dst, src2dst_w, it->second.c_str());
						IKAssert(cloned);
						return true;
					}
					else
						return false;
				};

	bool cloned_tree = ConstructBFS(src, dst, CloneNode);

	if (cloned_tree)
	{
		KINA_Initialize(*dst);
		FK_Update<false>(*dst);
	}
	else
	{
		IKAssert(NULL);
	}
	return cloned_tree;
}



void CArtiBodyTree::KINA_Initialize(CArtiBodyNode* root)
{
	auto onEnterBody = [](CArtiBodyNode* node_this)
					{
						node_this->m_kinalst.clear();
						node_this->m_kinalst.push_back(node_this);
					};

	auto onLeaveBody = [](CArtiBodyNode* node_this)
					{
						auto& this_kinalst = node_this->m_kinalst;
						for (auto node_child = node_this->GetFirstChild();
							node_child != NULL;
							node_child = node_child->GetNextSibling())
						{
							const auto& child_kinalst = node_child->m_kinalst;
							this_kinalst.insert(this_kinalst.end(),
												child_kinalst.begin(),
												child_kinalst.end());
						}
						node_this->OnKINA_Initialize();
					};

	Tree<CArtiBodyNode>::TraverseDFS(root, onEnterBody, onLeaveBody);
}

int CArtiBodyTree::BodyCMP(const char* const pts_interest[], int n_interests, const CArtiBodyNode* root_s, const CArtiBodyNode* root_d, HBODY* err_nodes, Real* err_oris)
{
	std::set<std::string> pts;
	for (int i_interest = 0; i_interest < n_interests; i_interest ++)
		pts.insert(pts_interest[i_interest]);


	int n_err_nodes_cap = n_interests;
	int n_err_nodes = 0;
	auto NodeEQ = [&n_err_nodes, n_err_nodes_cap, &pts, err_nodes, err_oris](const CArtiBodyNode* node_s, const CArtiBodyNode* node_d) -> bool
				{
					auto tmEQ = [&]() -> bool
						{
							const Transform* tm_s = node_s->GetTransformLocal2Parent();
							const Transform* tm_d = node_d->GetTransformLocal2Parent();
							Eigen::Quaternionr ori_s = Transform::getRotation_q(tm_s);
							Eigen::Quaternionr ori_d = Transform::getRotation_q(tm_d);
							Eigen::Vector3r tt_s = tm_s->getTranslation();
							Eigen::Vector3r tt_d = tm_d->getTranslation();

							Real norm_tt_s = tt_s.norm();
							Real norm_tt_d = tt_d.norm();
							const Real cos_epsilon = (Real)cos(M_PI*(Real)7/(Real)180);
							bool zero_tt_s = (norm_tt_s < c_epsilon);
							bool zero_tt_d = (norm_tt_d < c_epsilon);
							LOGIKVar(LogInfoCharPtr, node_s->GetName_c());
							bool ori_eq = ori_s.isApprox(ori_d);
							Real err_tt = tt_s.dot(tt_d);
							Real cos_err = (Real)1;
							bool tt_eq = (zero_tt_s == zero_tt_d)
								&& (zero_tt_s || cos_epsilon < (cos_err = err_tt / (norm_tt_d*norm_tt_s))); // cos_epsilon < cos_err -> epsilon > err
							bool eq = ori_eq && tt_eq;
							if (!tt_eq)
							{
								err_oris[n_err_nodes] = rad2deg(wrap_pi(acos(cos_err)));
							}
							// LOGIKVar(LogInfoBool, ori_eq);
							// LOGIKVar(LogInfoBool, tt_eq);
							// LOGIKVar(LogInfoReal, err_tt);
							// LOGIKVar(LogInfoReal, cos_err);
							// LOGIKVar(LogInfoReal, norm_tt_d);
							// LOGIKVar(LogInfoReal, norm_tt_s);
							// std::stringstream tt_s_str;
							// tt_s_str << tt_s;
							// LOGIKVar(LogInfoCharPtr, tt_s_str.str().c_str());
							// std::stringstream tt_d_str;
							// tt_d_str << tt_d;
							// LOGIKVar(LogInfoCharPtr, tt_d_str.str().c_str());
							// LOGIKFlush();
							return eq;
						};
					auto nameEQ = [&]() -> bool
						{
							return 0 == strcmp(node_s->GetName_c(), node_d->GetName_c());
						};
					auto is_an_interest = [&]() -> bool
						{
							return pts.end() != pts.find(node_s->GetName_c())
								|| pts.end() != pts.find(node_d->GetName_c());
						};
					bool eq_node = !is_an_interest() || (nameEQ() && tmEQ());
					if (!eq_node)
					{
						if (n_err_nodes < n_err_nodes_cap)
							err_nodes[n_err_nodes ++] = CAST_2HBODY(const_cast<CArtiBodyNode*>(node_d));
					}
					return true;
				};
	auto NodeCMP = [](const CArtiBodyNode* root_s, const CArtiBodyNode* root_d) -> bool
				{
					return strcmp(root_s->GetName_c(), root_d->GetName_c()) < 0;
				};
	Super::TraverseBFS_Bound(root_s, root_d, NodeCMP, NodeEQ);
	return n_err_nodes;
}

void CArtiBodyTree::Body_T_Test(const CArtiBodyNode* body, const Eigen::Vector3r& dir_up
					, const std::vector<std::string>& names_interest
					, int part_body_idx_range[parts_total][2]
					, Real err[])
{
	std::map<std::string, const CArtiBodyNode*> name2body;
	auto onEnterBody = [&name2body](const CArtiBodyNode* node_this) -> bool
					{
						name2body[node_this->GetName_c()] = node_this;
						return true;
					};

	auto onLeaveBody = [](const CArtiBodyNode* node_this) -> bool
					{
						return true;
					};
	Super::TraverseDFS(body, onEnterBody, onLeaveBody);

	auto Error = [](const Eigen::Vector3r& vec_seg, const Eigen::Vector3r& dir_standard) -> Real //[0 180]
				{
					Real vec_seg_norm = vec_seg.norm();
			 		Real cos_alpha = vec_seg_norm > c_100epsilon
	 						? abs(vec_seg.dot(dir_standard))/vec_seg_norm
	 						: (Real)1;
		 			return rad2deg(acos(cos_alpha));
				};

	int idx_right_arm_base = part_body_idx_range[right_arm][0];
	int idx_left_arm_base = part_body_idx_range[left_arm][0];

	Eigen::Vector3r dir_right =	(name2body[names_interest[idx_right_arm_base]]->GetTransformLocal2World()->getTranslation()
								- name2body[names_interest[idx_left_arm_base]]->GetTransformLocal2World()->getTranslation())
								.normalized();
	Eigen::Vector3r dir_front = dir_up.cross(dir_right);
	dir_right = dir_front.cross(dir_up);

	PART parts[] = {spine, left_leg, right_leg, left_arm, right_arm};
	Eigen::Vector3r dirs[] = {dir_up, -dir_up, -dir_up, -dir_right, dir_right};
	for (auto part : parts)
	{
		for (int i_body = part_body_idx_range[part][0]; i_body < part_body_idx_range[part][1]; i_body ++)
		{
			const CArtiBodyNode* body = name2body[names_interest[i_body]];
			const CArtiBodyNode* body_p = body->GetParent();
			Eigen::Vector3r vec_seg = body->GetTransformLocal2World()->getTranslation()
										- body_p->GetTransformLocal2World()->getTranslation();
			err[i_body] = Error(vec_seg, dirs[part]);
		}
	}
}

int CArtiBodyTree::GetBodies(const CArtiBodyNode* root
							, const std::list<std::string>& names
							, std::list<const CArtiBodyNode*>& nodes)
{
	std::set<std::string> set_names(names.begin(), names.end());
	int n_bodies_interests = 0;
	for (auto node : root->m_kinalst)
	{
		bool is_a_interest = (set_names.end() != set_names.find(node->GetName_c()));
		if (is_a_interest)
		{
			nodes.push_back(node);
			n_bodies_interests ++;
		}
	}
	return n_bodies_interests;
}

#ifdef _DEBUG
void CArtiBodyTree::Connect(CArtiBodyNode* from, CArtiBodyNode* to, CNN type)
{
	assert(from->c_type == to->c_type);
	Super::Connect(from, to, type);
}
#endif

CArtiBodyNode::CArtiBodyNode(const wchar_t *name, BODY_TYPE type, TM_TYPE jtmflag)
					: TreeNode<CArtiBodyNode>()
					, c_type(type)
					, c_jtmflag(jtmflag)
{
	m_namew = name;
	std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
	m_namec = converter.to_bytes(m_namew);
}

CArtiBodyNode::CArtiBodyNode(const char *name, BODY_TYPE type, TM_TYPE jtmflag)
					: TreeNode<CArtiBodyNode>()
					, c_type(type)
					, c_jtmflag(jtmflag)
{
	m_namec = name;
	std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
	m_namew = converter.from_bytes(m_namec);
}

CArtiBodyNode::~CArtiBodyNode()
{
}

