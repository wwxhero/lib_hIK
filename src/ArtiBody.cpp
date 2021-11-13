#include "pch.h"
#include <queue>
#include <stack>
#include <map>
#include <set>
#include <sstream>
#include <math.h>
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


	const wchar_t* name_dst = (NULL == name_dst_opt ? src->GetName_w() : name_dst_opt);

	*dst = CreateAnimNode(name_dst, &l2p_0_tm);
	return NULL != *dst;
}

bool CArtiBodyTree::CloneNode_bvh(const CArtiBodyNode* src, CArtiBodyNode** dst, const wchar_t* name_dst_opt)
{
	*dst = NULL;
	CArtiBodyNode* src_parent = src->GetParent();
	bool jtm_copy = (src->c_type&sim);
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
			vec_to_mat3_normalized_sim(nor, rot_m);
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
				LOGIKVarErr(LogInfoCharPtr, msg.str().c_str());
			}
			LOGIKVar(LogInfoWCharPtr, src->GetName_w());
			LOGIKVar(LogInfoWCharPtr, name_dst_opt);
			LOGIKVar(LogInfoReal1x3, nor.data());
			LOGIKVar(LogInfoReal3x3, rot_m.data());
#endif
		}
		else
			rot_q = Eigen::Quaternionr::Identity();

		_TRANSFORM tm = {
			{1, 1, 1},
			{rot_q.w(), rot_q.x(), rot_q.y(), rot_q.z()},
			{tt_this.x(), tt_this.y(), tt_this.z()}
		};
		bool jtm_copy = (src->c_type&sim);
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


void CArtiBodyTree::Destroy(CArtiBodyNode* node)
{
	Tree<CArtiBodyNode>::Destroy(node);
	CArtiBodyNode* root = node;
	for (CArtiBodyNode* root_p = root->GetParent()
		; NULL != root_p
		; root = root_p);
	bool is_sub_root = (node != root);
	if (is_sub_root)
		KINA_Initialize(root);
}



void CArtiBodyTree::Body_T_Test(const CArtiBodyNode* body, const Eigen::Vector3r& dir_up, const Eigen::Vector3r& dir_forward
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
					Real cos_alpha = (vec_seg_norm > c_epsilonsqrt)
							? vec_seg.dot(dir_standard)/vec_seg_norm
							: (Real)1;
					cos_alpha = std::min<Real>(1, std::max<Real>(-1, cos_alpha));
					return rad2deg(acos(cos_alpha));
				};
	const Real Error_max = (Real)180;

	int idx_right_arm_base = part_body_idx_range[right_arm][0];
	int idx_left_arm_base = part_body_idx_range[left_arm][0];

	// Eigen::Vector3r dir_right =	(name2body[names_interest[idx_right_arm_base]]->GetTransformLocal2World()->getTranslation()
	// 							- name2body[names_interest[idx_left_arm_base]]->GetTransformLocal2World()->getTranslation())
	// 							.normalized();
	// Eigen::Vector3r dir_front = dir_up.cross(dir_right);
	// dir_right = dir_front.cross(dir_up);

	Eigen::Vector3r dir_right = dir_forward.cross(dir_up);

	PART parts[] = {spine, left_leg, right_leg, left_arm, right_arm};
	Eigen::Vector3r dirs[] = {dir_up, -dir_up, -dir_up, -dir_right, dir_right};
	for (auto part : parts)
	{
		for (int i_body = part_body_idx_range[part][0]; i_body < part_body_idx_range[part][1]; i_body ++)
		{
			const CArtiBodyNode* body = name2body[names_interest[i_body]];
			Eigen::Vector3r vec_seg = body->GetTransformLocal2World()->getTranslation();
			const CArtiBodyNode* body_p = body->GetParent();
			if (NULL != body_p)
				vec_seg -= body_p->GetTransformLocal2World()->getTranslation();
			err[i_body] = Error(vec_seg, dirs[part]);
		}
	}

	int i_body_hip = part_body_idx_range[spine][0];
	const CArtiBodyNode* body_root = name2body[names_interest[i_body_hip]];
	const Transform* l2w = body_root->GetTransformLocal2World();
	bool bvh_root = (l2w->getTranslation().norm() < c_epsilon);
	if (!bvh_root)
		err[i_body_hip] = Error_max;
}

void CArtiBodyTree::Body_EQ_Test(const CArtiBodyNode* body_s
								, const CArtiBodyNode* body_d
								, const std::vector<std::string>& pts_interest
								, Real err[])
{
	const Real Error_max = (Real)180;

	typedef std::pair<const CArtiBodyNode*, const CArtiBodyNode*> Bound;
	std::map<std::string, Bound> name2Bound;

	std::set<std::string> pts;
	for (auto name : pts_interest)
		pts.insert(name);


	auto OnBound = [&pts, &name2Bound](const CArtiBodyNode* node_s, const CArtiBodyNode* node_d) -> bool
				{
					std::string name_s = node_s->GetName_c();
					std::string name_d = node_d->GetName_c();
					bool is_an_interest = ((name_d == name_s)
										&& (pts.end() != pts.find(name_s)));
					if (is_an_interest)
					{
						name2Bound[name_s] = std::make_pair(node_s, node_d);
					}
					return true;
				};
	auto NodeCMP = [](const CArtiBodyNode* root_s, const CArtiBodyNode* root_d) -> bool
				{
					return strcmp(root_s->GetName_c(), root_d->GetName_c()) < 0;
				};
	Super::TraverseBFS_Bound(body_s, body_d, NodeCMP, OnBound);

	auto Bone_Axis = [](const CArtiBodyNode* body) -> Eigen::Vector3r
				{
					Eigen::Vector3r axis = body->GetTransformLocal2World()->getTranslation();
					const CArtiBodyNode* body_p = body->GetParent();
					if (NULL != body_p)
						axis -= body_p->GetTransformLocal2World()->getTranslation();
					return axis;
				};

	auto Axis_Err = [](const Eigen::Vector3r& vec_s, const Eigen::Vector3r& vec_d) -> Real
				{
					Real vec_norm_s = vec_s.norm();
					Real vec_norm_d = vec_d.norm();
					Real cos_alpha = (vec_norm_s > c_epsilonsqrt && vec_norm_d > c_epsilonsqrt)
									? vec_s.dot(vec_d)/(vec_norm_s*vec_norm_d)
									: (Real)1;
					cos_alpha = std::min<Real>(1, std::max<Real>(-1, cos_alpha));
					return rad2deg(acos(cos_alpha));
				};

	int n_errs = (int)pts_interest.size();
	for (int i_err = 0; i_err < n_errs; i_err ++)
	{
		auto it_bnd = name2Bound.find(pts_interest[i_err]);
		if (name2Bound.end() == it_bnd)
			err[i_err] = Error_max;
		else
		{
			Eigen::Vector3r axis_s = Bone_Axis(it_bnd->second.first);
			Eigen::Vector3r axis_d = Bone_Axis(it_bnd->second.second);
			err[i_err] = Axis_Err(axis_s, axis_d);
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

