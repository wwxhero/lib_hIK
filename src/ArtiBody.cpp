#include "pch.h"
#include <queue>
#include <stack>
#include <map>
#include "articulated_body.h"
#include "ArtiBody.h"
#include "ik_logger.h"
#include <sstream>


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



bool CArtiBodyTree::CloneNode_htr(const CArtiBodyNode* src, CArtiBodyNode** dst, const wchar_t* name_dst_opt)
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
		const Transform* tm_this = src->GetTransformLocal2World();
		const Eigen::Vector3r& tt_this = tm_this->getTranslation();

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

		auto onSearchBody = [&nor, tt_this, GetDir](const CArtiBodyNode* node) -> bool
			{
				Eigen::Vector3r tt_other = node->GetTransformLocal2World()->getTranslation();
				bool valid_dir = GetDir(tt_this, tt_other, nor);
				return valid_dir;
			};

		if (!(valid_nor = Tree<CArtiBodyNode>::SearchBFS(src, onSearchBody)))
		{
			while (!valid_nor
				&& (NULL != parent_src))
			{
				Eigen::Vector3r tt_other = parent_src->GetTransformLocal2World()->getTranslation();
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
		TM_TYPE tm_type = (is_root? t_tr : t_r);
		const wchar_t* name_dst = (NULL == name_dst_opt ? src->GetName_w() : name_dst_opt);
		*dst = CreateSimNode(name_dst, &tm, htr, tm_type, false);

		return NULL != *dst;
	}

}

bool CArtiBodyTree::CloneNode(const CArtiBodyNode* src, BODY_TYPE type, CArtiBodyNode** dst, const wchar_t* name_dst)
{
	bool ret = false;
	switch(type)
	{
		case fbx:
		{
			ret = CloneNode_fbx(src, dst, name_dst);
			break;
		}
		case bvh:
		{
			ret = CloneNode_bvh(src, dst, name_dst);
			break;
		}
		case htr:
		{
			ret = CloneNode_htr(src, dst, name_dst);
			break;
		}
	}
	return ret;
}

bool CArtiBodyTree::Clone(const CArtiBodyNode* src, CArtiBodyNode** dst, const wchar_t* (*a_matches)[2], int n_matches, bool src_on_match0)
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

	std::stack<CArtiBodyNode*> dstSTK;
	CArtiBodyNode* root_dst = NULL;
	const CArtiBodyNode* root_src = src;
	auto it = matches.find(root_src->GetName_w());

	IKAssert(it != matches.end());
	bool cloned_root = CArtiBodyTree::CloneNode(root_src, htr, &root_dst, it->second.c_str());

	dstSTK.push(root_dst); //root node is unconditionally a interest

	auto onEnterBody = [&dstSTK, &matches](const CArtiBodyNode* node_src) -> bool
					{
						auto it = matches.find(node_src->GetName_w());
						bool interest = (it != matches.end());
						CArtiBodyNode* node_dst = NULL;
						bool cloned = false;
						if ( interest
						  && (cloned = CArtiBodyTree::CloneNode(node_src, htr, &node_dst, it->second.c_str())))
						{
							CArtiBodyNode* node_dst_parent = dstSTK.top();
							CArtiBodyTree::Connect(node_dst_parent, node_dst, FIRSTCHD);
							dstSTK.push(node_dst);
						}
						return !interest
							|| cloned;
					};

	auto onLeaveBody = [&dstSTK, &matches](const CArtiBodyNode* node_src) -> bool
					{
						auto it = matches.find(node_src->GetName_w());
						bool interest = (matches.end() != it);
						assert(!interest || (it->second == (dstSTK.top()->GetName_w())));
						if (interest)
							dstSTK.pop();
						return true;
					};

	bool cloned_tree = cloned_root;

	for (const CArtiBodyNode* sub_root = root_src->GetFirstChild()
		; cloned_tree && NULL != sub_root
		; sub_root = sub_root->GetNextSibling())
		cloned_tree = (Tree<CArtiBodyNode>::TraverseDFS(sub_root, onEnterBody, onLeaveBody));

	if (cloned_tree)
	{
		KINA_Initialize(root_dst);
		FK_Update(root_dst);
		*dst = root_dst;
	}
	else
		*dst = NULL;
	return cloned_tree;
}

bool CArtiBodyTree::Clone(const CArtiBodyNode* src, BODY_TYPE type, CArtiBodyNode** dst)
{
	typedef std::pair<const CArtiBodyNode*, CArtiBodyNode*> Bound;
	//traverse the bvh herachical structure
	//	to create an articulated body with the given posture
	std::queue<Bound> queBFS;
	const CArtiBodyNode* root_src = src;
	CArtiBodyNode* root_dst = NULL;
	bool cloned = CArtiBodyTree::CloneNode(root_src, type, &root_dst);
	if (cloned)
	{
		Bound root = std::make_pair(
			root_src,
			root_dst
		);
		queBFS.push(root);
		while (!queBFS.empty()
			&& cloned)
		{
			Bound pair = queBFS.front();
			const CArtiBodyNode* body_src = pair.first;
			CArtiBodyNode* body_dst = pair.second;
			CNN cnn = FIRSTCHD;
			CArtiBodyNode* b_this = body_dst;
			for (const CArtiBodyNode* child_body_src = body_src->GetFirstChild()
				; NULL != child_body_src && cloned
				; child_body_src = child_body_src->GetNextSibling())
			{
				CArtiBodyNode* child_body_dst = NULL;
				cloned = CArtiBodyTree::CloneNode(child_body_src, type, &child_body_dst);
				if (cloned)
				{
					Bound child = std::make_pair(
						child_body_src,
						child_body_dst
					);
					queBFS.push(child);
					CArtiBodyNode* b_next = child_body_dst;
					CArtiBodyTree::Connect(b_this, b_next, cnn);
					cnn = NEXTSIB;
					b_this = b_next;
				}
			}
			queBFS.pop();
		}
	}

	if (cloned)
	{
		KINA_Initialize(root_dst);
		FK_Update(root_dst);
		*dst = root_dst;
	}
	else
	{
		if (NULL != root_dst)
			Destroy(root_dst);
	}

	return cloned;

}

void CArtiBodyTree::FK_Update(CArtiBodyNode* root)
{
	bool is_an_anim = (root->c_type&anim);
	if (is_an_anim)
	{
		for (auto body : root->m_kinalst)
			static_cast<CArtiBodyNode_anim*>(body)->FK_UpdateNode();
	}
	else // not an anim
	{
		for (auto body : root->m_kinalst)
		{
			switch (body->c_jtmflag)
			{
				case t_r:
					static_cast<CArtiBodyNode_sim_r*>(body)->FK_UpdateNode();
					break;
				case t_tr:
					static_cast<CArtiBodyNode_sim_tr*>(body)->FK_UpdateNode();
					break;
			}
		}
	}
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

void CArtiBodyTree::Destroy(CArtiBodyNode* root)
{
	auto onEnterBody = [](CArtiBodyNode* node_this)
					{
					};

	auto onLeaveBody = [](CArtiBodyNode* node_this)
					{
						delete node_this;
					};

	Tree<CArtiBodyNode>::TraverseDFS(root, onEnterBody, onLeaveBody);
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

