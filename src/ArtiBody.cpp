#include "pch.h"
#include <queue>
#include <stack>
#include <map>
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
	const wchar_t* name_dst = (NULL == name_dst_opt ? src->GetName_w() : name_dst_opt);
	bool is_root = (NULL == src_parent);
	if (is_root)
	{
		_TRANSFORM l2p_0_tm = {
			{1, 1, 1},
			{1, 0, 0, 0},
			{0, 0, 0}
		};
		*dst = CreateSimNode(name_dst, &l2p_0_tm, bvh, t_tr, true);
	}
	else
	{
		const Transform* l2w_parent = src_parent->GetTransformLocal2World();
		const Transform* l2w_this = src->GetTransformLocal2World();
		auto offset = l2w_this->getTranslation() - l2w_parent->getTranslation();
		_TRANSFORM l2p_0_tm = {
			{1, 1, 1},
			{1, 0, 0, 0},
			{offset.x(), offset.y(), offset.z()}
		};
		*dst = CreateSimNode(name_dst, &l2p_0_tm, bvh, t_r, true);
	}
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
		// entity node and hip node are tr nodes: translation + rotation
		TM_TYPE tm_type = ((is_root || NULL == parent_src->GetParent()) ? t_tr : t_r);
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

	bool cloned_tree = Construct(src, dst, CloneNode);

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

int CArtiBodyTree::BodyCMP(const CArtiBodyNode* root_s, const CArtiBodyNode* root_d, HBODY* err_nodes, int n_err_nodes_cap)
{
	std::queue<const CArtiBodyNode*> que_s;
	std::queue<const CArtiBodyNode*> que_d;
	int n_err_nodes = 0;
	auto NodeEQ = [&n_err_nodes, n_err_nodes_cap, err_nodes](const CArtiBodyNode* node_s, const CArtiBodyNode* node_d) -> bool
				{
					auto oriEQ = [&]() -> bool
						{
							const Transform* tm_s = node_s->GetTransformLocal2Parent();
							const Transform* tm_d = node_d->GetTransformLocal2Parent();
							Eigen::Quaternionr ori_s = Transform::getRotation_q(tm_s);
							Eigen::Quaternionr ori_d = Transform::getRotation_q(tm_d);
							Eigen::Vector3r tt_s = tm_s->getTranslation();
							Eigen::Vector3r tt_d = tm_d->getTranslation();
							Real norm_tt_s = tt_s.norm();
							Real norm_tt_d = tt_d.norm();
							const Real cos_epsilon = (Real)cos(M_PI*(Real)5/(Real)180);
							bool zero_tt_s = (norm_tt_s < c_epsilon);
							bool zero_tt_d = (norm_tt_d < c_epsilon);
							LOGIKVar(LogInfoCharPtr, node_s->GetName_c());
							bool ori_eq = ori_s.isApprox(ori_d);
							Real err_tt = tt_s.dot(tt_d);
							Real cos_err = (Real)1;
							bool tt_eq = (zero_tt_s == zero_tt_d)
								&& (zero_tt_s || cos_epsilon < (cos_err = err_tt / (norm_tt_d*norm_tt_s))); // cos_epsilon < cos_err -> epsilon > err
							bool eq = ori_eq && tt_eq;
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
					bool eq_node = nameEQ() && oriEQ();
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
	que_s.push(root_s);
	que_d.push(root_d);
	bool body_eq = true;
	while ((body_eq = (body_eq && (que_s.empty() == que_d.empty())))
		&& !que_s.empty())
	{
		auto node_s = que_s.front();
		auto node_d = que_d.front();
		body_eq = NodeEQ(node_s, node_d);
		std::list<const CArtiBodyNode*> children_s;
		std::list<const CArtiBodyNode*> children_d;
		const CArtiBodyNode *child_s, *child_d;
		for (child_s = node_s->GetFirstChild(), child_d = node_d->GetFirstChild()
			; (body_eq = (body_eq && ((NULL == child_s) == (NULL == child_d))))
				&& NULL != child_s
			; child_s = child_s->GetNextSibling(), child_d = child_d->GetNextSibling())
		{
			children_s.push_back(child_s);
			children_d.push_back(child_d);
		}
		children_s.sort(NodeCMP);
		children_d.sort(NodeCMP);
		for (auto child_s : children_s)
			que_s.push(child_s);
		for (auto child_d : children_d)
			que_d.push(child_d);
		que_s.pop();
		que_d.pop();
	}
	return n_err_nodes;
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

