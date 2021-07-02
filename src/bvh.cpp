#include "pch.h"
#pragma push_macro("new")
#undef new
#include <Eigen/Geometry>
#pragma pop_macro("new")
#include <iostream>
#include <stack>
#include <queue>
#include "bvh.h"
#include "bvh11_helper.hpp"
#include "articulated_body.h"
#include "motion_pipeline.h"
#include "fk_joint.h"
#include "handle_helper.hpp"


#define ZERO_ENTITY_TT_HOMO
#define ZERO_ENTITY_TT_CROSS

const double epsilon = 1e-5;

#pragma warning( push )
#pragma warning( disable : 4838 )
#pragma warning( disable : 4244 )

typedef std::shared_ptr<const bvh11::Joint> Joint_bvh_ptr;
typedef std::pair<Joint_bvh_ptr, HBODY> Bound;

HBODY create_arti_body(bvh11::BvhObject& bvh, Joint_bvh_ptr j_bvh, int frame)
{
	auto name = j_bvh->name().c_str();
	auto tm_bvh = bvh.GetTransformationRelativeToParent(j_bvh, frame);
	Eigen::Quaterniond rq(tm_bvh.linear());
	Eigen::Vector3d tt;
#ifdef ZERO_ENTITY_TT_HOMO
	if (nullptr == j_bvh->parent())
		tt = Eigen::Vector3d::Zero();
	else
		tt = tm_bvh.translation();
#else
	tt = tm_bvh.translation();
#endif

	_TRANSFORM tm_hik = {
		{1, 1, 1},
		{rq.w(), rq.x(), rq.y(), rq.z()},
		{tt.x(), tt.y(), tt.z()}
	};
	auto b_hik = create_tree_body_node_c(name, &tm_hik);
#ifdef ZERO_ENTITY_TT_HOMO
	assert(nullptr != j_bvh->parent()
		|| tt.norm() < epsilon);
#endif
	return b_hik;
}

HBODY create_arti_body_as_rest_bvh_pose(bvh11::BvhObject& bvh, Joint_bvh_ptr j_bvh, int frame)
{
	auto name = j_bvh->name().c_str();
	auto tm_bvh = bvh.GetTransformation(j_bvh, frame);
	Eigen::Affine3d tm_parent_bvh;
	auto j_parent_bvh = j_bvh->parent();
	if (nullptr == j_parent_bvh)
	{
#if defined ZERO_ENTITY_TT_CROSS
		tm_parent_bvh.linear() = Eigen::Matrix3d::Identity();
		tm_parent_bvh.translation() = tm_bvh.translation();
#else
		tm_parent_bvh = Eigen::Affine3d::Identity();
#endif
	}
	else
	{
		tm_parent_bvh = bvh.GetTransformation(j_parent_bvh, frame);
	}

	Eigen::Vector3d tt = tm_bvh.translation() - tm_parent_bvh.translation();
	_TRANSFORM tm_hik = {
		{1, 1, 1},
		{1, 0, 0, 0},
		{tt.x(), tt.y(), tt.z()}
	};
	auto b_hik = create_tree_body_node_c(name, &tm_hik);
	assert(nullptr != j_parent_bvh
		|| tt.norm() < epsilon);
	return b_hik;
}


HBODY createArticulatedBody(bvh11::BvhObject& bvh, int frame, bool asRestBvhPose)
{
	//traverse the bvh herachical structure
	//	to create an articulated body with the given posture
	bool reset_posture = (frame < 0);
	std::queue<Bound> queBFS;
	auto root_j_bvh = bvh.root_joint();
	auto root_b_hik = asRestBvhPose
		? create_arti_body_as_rest_bvh_pose(bvh, root_j_bvh, frame)
		: create_arti_body(bvh, root_j_bvh, frame);
	Bound root = std::make_pair(
		root_j_bvh,
		root_b_hik
	);
	queBFS.push(root);
	while (!queBFS.empty())
	{
		auto pair = queBFS.front();
		auto j_bvh = pair.first;
		auto b_hik = pair.second;
		CNN cnn = FIRSTCHD;
		auto& rb_this = b_hik;
		for (auto j_bvh_child : j_bvh->children())
		{
			auto b_hik_child = asRestBvhPose
				? create_arti_body_as_rest_bvh_pose(bvh, j_bvh_child, frame)
				: create_arti_body(bvh, j_bvh_child, frame);
			Bound child = std::make_pair(
				j_bvh_child,
				b_hik_child
			);
			queBFS.push(child);
			auto& rb_next = b_hik_child;
			cnn_arti_body(rb_this, rb_next, cnn);
			cnn = NEXTSIB;
			rb_this = rb_next;
		}
		queBFS.pop();
	}
	initialize_kina(root_b_hik);
	update_fk(root_b_hik);
	return root_b_hik;
}




inline void printArtName(const char* name, int n_indent)
{
	std::string item;
	for (int i_indent = 0
		; i_indent < n_indent
		; i_indent++)
		item += "\t";
	item += name;
	std::cout << item.c_str() << std::endl;
}

inline void printBoundName(Bound bnd, int n_indent)
{
	std::string item;
	for (int i_indent = 0
		; i_indent < n_indent
		; i_indent++)
		item += "\t";
	std::string itemBvh(item);
	itemBvh += bnd.first->name();
	std::string itemBody(item);
	itemBody += body_name_c(bnd.second);
	std::cout << itemBvh.c_str() << std::endl;
	std::cout << itemBody.c_str() << std::endl;
}

inline bool TransformEq(const _TRANSFORM& tm_hik, const Eigen::Affine3d& tm_bvh)
{
	Eigen::Matrix3d linear_tm_bvh = tm_bvh.linear();
	Eigen::Vector3d tt_tm_bvh = tm_bvh.translation();

	Eigen::Vector3d s(tm_hik.s.x, tm_hik.s.y, tm_hik.s.z);
	Eigen::Matrix3d r(Eigen::Quaterniond(tm_hik.r.w, tm_hik.r.x, tm_hik.r.y, tm_hik.r.z));
	Eigen::Matrix3d linear_tm_hik = r * s.asDiagonal();
	Eigen::Vector3d tt_tm_hik(tm_hik.tt.x, tm_hik.tt.y, tm_hik.tt.z);

	Eigen::Matrix3d diff_linear = linear_tm_bvh.inverse() * linear_tm_bvh;
	double abs_diff_linear = (diff_linear - Eigen::Matrix3d::Identity()).norm();
	bool linear_eq = (-epsilon < abs_diff_linear
		&& abs_diff_linear < epsilon);
	double diff_tt = (tt_tm_bvh - tt_tm_hik).norm();
	bool tt_eq = (-epsilon < diff_tt
		&& diff_tt < epsilon);
	return linear_eq && tt_eq;
}

inline bool BoundEQ(Bound bnd, const bvh11::BvhObject& bvh, int frame)
{
	auto& joint_bvh = bnd.first;
	auto& body_hik = bnd.second;
	auto& name_bvh = joint_bvh->name();
	auto name_body = body_name_c(body_hik);
	bool name_eq = (name_bvh == name_body);
	Eigen::Affine3d tm_bvh = bvh.GetTransformation(joint_bvh, frame);

	_TRANSFORM tm_hik;
	get_body_transform_l2w(body_hik, &tm_hik);

	return name_eq && TransformEq(tm_hik, tm_bvh);
}

inline bool BoundResetAsBVHRest(Bound bnd, const bvh11::BvhObject& bvh, int frame)
{
	auto& joint_bvh = bnd.first;
	auto& body_hik = bnd.second;
	auto& name_bvh = joint_bvh->name();
	auto name_body = body_name_c(body_hik);
	bool name_eq = (name_bvh == name_body);
	Eigen::Affine3d tm_bvh = bvh.GetTransformation(joint_bvh, frame);
	auto joint_parent_bvh = joint_bvh->parent();
	Eigen::Affine3d tm_parent_bvh;
	if (nullptr == joint_parent_bvh)
	{
		tm_parent_bvh.linear() = Eigen::Matrix3d::Identity();
		tm_parent_bvh.translation() = tm_bvh.translation();
	}
	else
		tm_parent_bvh = bvh.GetTransformation(joint_parent_bvh, frame);
	Eigen::Vector3d tt_tm_bvh_l = tm_bvh.translation() - tm_parent_bvh.translation();


	_TRANSFORM tm_hik_l;
	get_body_transform_l2p(body_hik, &tm_hik_l);
	Eigen::Vector3d s(tm_hik_l.s.x, tm_hik_l.s.y, tm_hik_l.s.z);
	Eigen::Vector3d tt_tm_hik_l(tm_hik_l.tt.x, tm_hik_l.tt.y, tm_hik_l.tt.z);

	Real diff_s = (s - Eigen::Vector3d::Ones()).norm();
	Real diff_r = Eigen::Vector4d(tm_hik_l.r.w - 1, tm_hik_l.r.x, tm_hik_l.r.y, tm_hik_l.r.z).norm();
	Real diff_tt = (tt_tm_bvh_l - tt_tm_hik_l).norm();
	bool linear_id = (-epsilon < diff_s
		&&	diff_s < epsilon
		&&-epsilon < diff_r
		&&  diff_r < epsilon);
	bool tt_eq = (-epsilon < diff_tt
		&& diff_tt < epsilon);

	return name_eq && linear_id && tt_eq;
}

template<typename LAMaccessEnter, typename LAMaccessLeave>
inline void TraverseDFS_botree_nonrecur(HBODY root, LAMaccessEnter OnEnterBody, LAMaccessLeave OnLeaveBody)
{
	assert(VALID_HANDLE(root));
	typedef struct _EDGE
	{
		HBODY body_this;
		HBODY body_child;
	} EDGE;
	std::stack<EDGE> stkDFS;
	stkDFS.push({ root, get_first_child_body(root) });
	//printArtName(body_name_w(root), 0);
	OnEnterBody(root);
	while (!stkDFS.empty())
	{
		EDGE &edge = stkDFS.top();
		// size_t n_indent = stkDFS.size();
		if (!VALID_HANDLE(edge.body_child))
		{
			stkDFS.pop();
			OnLeaveBody(edge.body_this);
		}
		else
		{
			//printArtName(body_name_w(edge.body_child), n_indent);
			OnEnterBody(edge.body_child);
			HBODY body_grandchild = get_first_child_body(edge.body_child);
			HBODY body_nextchild = get_next_sibling_body(edge.body_child);
			stkDFS.push({ edge.body_child, body_grandchild });
			edge.body_child = body_nextchild;
		}
	}
}

template<typename LAMaccessEnter, typename LAMaccessLeave>
inline void TraverseDFS_motree_nonrecur(HMOTIONNODE root, LAMaccessEnter OnEnterBody, LAMaccessLeave OnLeaveBody)
{
	assert(VALID_HANDLE(root));
	typedef struct _EDGE
	{
		HMOTIONNODE body_this;
		HMOTIONNODE body_child;
	} EDGE;
	std::stack<EDGE> stkDFS;
	stkDFS.push({ root, get_first_child_mo_node(root) });
	//printArtName(body_name_w(root), 0);
	OnEnterBody(root);
	while (!stkDFS.empty())
	{
		EDGE &edge = stkDFS.top();
		size_t n_indent = stkDFS.size();
		if (!VALID_HANDLE(edge.body_child))
		{
			stkDFS.pop();
			OnLeaveBody(edge.body_this);
		}
		else
		{
			//printArtName(body_name_w(edge.body_child), n_indent);
			OnEnterBody(edge.body_child);
			HMOTIONNODE body_grandchild = get_first_child_mo_node(edge.body_child);
			HMOTIONNODE body_nextchild = get_next_sibling_mo_node(edge.body_child);
			stkDFS.push({ edge.body_child, body_grandchild });
			edge.body_child = body_nextchild;
		}
	}
}


template<typename LAMaccessEnter, typename LAMaccessLeave>
inline void TraverseDFS_boundtree_recur(Bound bound_this, LAMaccessEnter OnEnterBound, LAMaccessLeave OnLeaveBound)
{
	OnEnterBound(bound_this);
	auto bvh_this = bound_this.first;
	auto body_this = bound_this.second;
	const auto& children_bvh_this = bvh_this->children();
	auto it_bvh_next = children_bvh_this.begin();
	auto body_next = get_first_child_body(body_this);
	bool proceed = (it_bvh_next != children_bvh_this.end());
	assert((proceed)
		== VALID_HANDLE(body_next));
	while (proceed)
	{
		Bound bound_next = std::make_pair(*it_bvh_next, body_next);
		TraverseDFS_boundtree_recur(bound_next, OnEnterBound, OnLeaveBound);
		it_bvh_next++;
		body_next = get_next_sibling_body(body_next);
		proceed = (it_bvh_next != children_bvh_this.end());
		assert((proceed)
			== VALID_HANDLE(body_next));
	}
	OnLeaveBound(bound_this);
}

void updateHeader(bvh11::BvhObject& bvh, HBODY body)
{
	//update the header part of the BVH file
	//	, the given body is in BVH rest posture
	auto lam_onEnter = [&bvh = std::as_const(bvh)](Bound b_this)
	{
		Joint_bvh_ptr joint_this = b_this.first;
		HBODY body_this = b_this.second;
		_TRANSFORM tm_l2p;
		get_body_transform_l2p(body_this, &tm_l2p);
		const_cast<Eigen::Vector3d&>(joint_this->offset()) = Eigen::Vector3d(tm_l2p.tt.x, tm_l2p.tt.y, tm_l2p.tt.z);
	};
	auto lam_onLeave = [](Bound b_this)
	{
	};
	Bound root = std::make_pair(bvh.root_joint(), body);
	TraverseDFS_boundtree_recur(root, lam_onEnter, lam_onLeave);
}

inline bool verify_bound(Bound b_this, bool enter, const bvh11::BvhObject& bvh, int i_frame)
{
	Joint_bvh_ptr joint_bvh = b_this.first;
	HBODY body_hik = b_this.second;
	const std::string& name_bvh = joint_bvh->name();
	std::string name_hik(body_name_c(body_hik));

	auto tm_l2w_bvh = bvh.GetTransformation(joint_bvh, i_frame);
	_TRANSFORM tm_l2w_hik = { 0 };
	get_body_transform_l2w(body_hik, &tm_l2w_hik);

	auto tm_l2p_bvh = bvh.GetTransformationRelativeToParent(joint_bvh, i_frame);
	_TRANSFORM tm_l2p_hik = { 0 };
	get_body_transform_l2p(body_hik, &tm_l2p_hik);

	auto tm_l_bvh = bvh.GetLocalDeltaTM(joint_bvh, i_frame);
	_TRANSFORM tm_l_hik = { 0 };
	get_joint_transform(body_hik, &tm_l_hik);

	bool delta_l_eq = TransformEq(tm_l_hik, tm_l_bvh);
	bool l2p_eq = TransformEq(tm_l2p_hik, tm_l2p_bvh);
	bool l2w_eq = TransformEq(tm_l2w_hik, tm_l2w_bvh);
	bool name_eq = (name_bvh == name_hik);

	bool verified = (name_eq && delta_l_eq && l2p_eq && l2w_eq);
	if (!verified)
	{
		std::cout << "Enter" << enter
			<< "\t" << name_hik.c_str()
			<< ":\t delta_l_eq = " << delta_l_eq
			<< "\t l2p_eq = " << l2p_eq
			<< "\t l2w_eq = " << l2w_eq
			<< "\t name_eq = " << name_eq
			<< "\t frame = " << i_frame
			<< std::endl;
	}
	return verified;
}

void pose(HBODY body_root, const bvh11::BvhObject& bvh, int i_frame)
{
	{
		auto lam_onEnter = [&bvh = std::as_const(bvh), i_frame](Bound b_this) -> void
		{
			Joint_bvh_ptr joint_bvh = b_this.first;
			HBODY body_hik = b_this.second;
			Eigen::Affine3d delta_l = bvh.GetLocalDeltaTM(joint_bvh, i_frame);
			Eigen::Quaterniond r(delta_l.linear());
			Eigen::Vector3d tt(delta_l.translation());
			_TRANSFORM delta_l_tm = {
				{1, 1, 1}, //scale is trivial
				{r.w(), r.x(), r.y(), r.z()}, //rotation
				{tt.x(), tt.y(), tt.z()}, //trivial
			};
			set_joint_transform(body_hik, &delta_l_tm);
		};
		auto lam_onLeave = [&bvh = std::as_const(bvh), i_frame](Bound b_this) -> void
		{
		};
		Bound root = std::make_pair(bvh.root_joint(), body_root);
		TraverseDFS_boundtree_recur(root, lam_onEnter, lam_onLeave);
	}
	update_fk(body_root);
#if defined _DEBUG
	{
		//pose the articulated body with the posture for frame i_frame
		//	the articulated body should have same rest posture as bvh
		auto lam_onEnter = [&bvh = std::as_const(bvh), i_frame](Bound b_this) -> void
		{
			verify_bound(b_this, true, bvh, i_frame);
		};
		auto lam_onLeave = [&bvh = std::as_const(bvh), i_frame](Bound b_this) -> void
		{
			verify_bound(b_this, false, bvh, i_frame);
		};
		Bound root = std::make_pair(bvh.root_joint(), body_root);
		TraverseDFS_boundtree_recur(root, lam_onEnter, lam_onLeave);
	}
#endif
}

template<typename LAMaccessEnter, typename LAMaccessLeave>
inline void TraverseBFS_boundtree_norecur(Bound root, LAMaccessEnter OnEnterBound, LAMaccessLeave OnLeaveBound)
{
	std::queue<Bound> queBFS;
	queBFS.push(root);
	OnEnterBound(root);
	while (!queBFS.empty())
	{
		Bound b_this = queBFS.front();
		Joint_bvh_ptr joint_bvh = b_this.first;
		HBODY body_hik = b_this.second;
		auto& children_bvh = joint_bvh->children();
		auto it_bvh_child = children_bvh.begin();
		auto body_child = get_first_child_body(body_hik);
		for (
			; it_bvh_child != children_bvh.end()
			&& VALID_HANDLE(body_child)
			; it_bvh_child++,
			body_child = get_next_sibling_body(body_child))
		{
			Bound b_child = std::make_pair(*it_bvh_child, body_child);
			queBFS.push(b_child);
			OnEnterBound(b_child);
		}
		queBFS.pop();
		OnLeaveBound(b_this);
	}
}
// pose articulated body with the bvh and the frame
void pose_nonrecur(HBODY body_root, const bvh11::BvhObject& bvh, int i_frame, bool header_resetted)
{
	auto onEnterBound_pose = [&bvh, i_frame](Bound b_this)
	{
		Joint_bvh_ptr joint_bvh = b_this.first;
		HBODY body_hik = b_this.second;
		Eigen::Affine3d delta_l = bvh.GetLocalDeltaTM(joint_bvh, i_frame);
		Eigen::Quaterniond r(delta_l.linear());
		Eigen::Vector3d tt(delta_l.translation());
		_TRANSFORM delta_l_tm = {
			{1, 1, 1}, //scale is trivial
			{r.w(), r.x(), r.y(), r.z()}, //rotation
			{tt.x(), tt.y(), tt.z()}, //trivial
		};
		set_joint_transform(body_hik, &delta_l_tm);
	};
	auto onLeaveBound_pose = [](Bound b_this) {};

	Bound root = std::make_pair(bvh.root_joint(), body_root);
	TraverseBFS_boundtree_norecur(root, onEnterBound_pose, onLeaveBound_pose);

	update_fk(body_root);

	if (!header_resetted)
	{
		auto onEnterBound_verify = [](Bound b_this)
		{
		};

		auto onLeaveBound_verify = [&bvh, i_frame](Bound b_this)
		{
			verify_bound(b_this, false, bvh, i_frame);
		};

		TraverseBFS_boundtree_norecur(root, onEnterBound_verify, onLeaveBound_verify);
	}
}

void updateBVHAnim(HBODY body_root, bvh11::BvhObject& bvh, int i_frame, bool header_resetted)
{
	auto onEnterBound_pose = [&bvh, i_frame](Bound b_this)
	{
		Joint_bvh_ptr joint_bvh = b_this.first;
		HBODY body_hik = b_this.second;
		_TRANSFORM delta_l_tm;
		get_joint_transform(body_hik, &delta_l_tm);

		Eigen::Affine3d delta_l;
		Eigen::Vector3d tt(delta_l_tm.tt.x,
			delta_l_tm.tt.y,
			delta_l_tm.tt.z);
		Eigen::Quaterniond r(delta_l_tm.r.w,
			delta_l_tm.r.x,
			delta_l_tm.r.y,
			delta_l_tm.r.z);
		Eigen::Vector3d s(delta_l_tm.s.x,
			delta_l_tm.s.y,
			delta_l_tm.s.z);
		delta_l.fromPositionOrientationScale(tt, r, s);
		bvh.UpdateMotion(joint_bvh, delta_l, i_frame);

	};
	auto onLeaveBound_pose = [](Bound b_this) {};

	Bound root = std::make_pair(bvh.root_joint(), body_root);
	TraverseBFS_boundtree_norecur(root, onEnterBound_pose, onLeaveBound_pose);

	update_fk(body_root);

	if (header_resetted)
	{
		auto onEnterBound_verify = [](Bound b_this)
		{
		};

		auto onLeaveBound_verify = [&bvh, i_frame](Bound b_this)
		{
			verify_bound(b_this, true, bvh, i_frame);
		};

		TraverseBFS_boundtree_norecur(root, onEnterBound_verify, onLeaveBound_verify);
	}

}

bool ResetRestPose(bvh11::BvhObject& bvh, int t)
{
	int n_frames = bvh.frames();
	bool in_range = (-1 < t
		&& t < n_frames);
	if (!in_range)
		return false;
	HBODY h_driver = createArticulatedBody(bvh, -1, false); //t = -1: the rest posture in BVH file
#if defined _DEBUG
	{
		std::cout << "Bounds:" << std::endl;
		int n_indent = 1;
		auto lam_onEnter = [&n_indent, &bvh = std::as_const(bvh)](Bound b_this)
		{
			printBoundName(b_this, n_indent++);
			assert(BoundEQ(b_this, bvh, -1));
		};
		auto lam_onLeave = [&n_indent](Bound b_this)
		{
			n_indent--;
		};
		Bound root = std::make_pair(bvh.root_joint(), h_driver);
		TraverseDFS_boundtree_recur(root, lam_onEnter, lam_onLeave);
	}
#endif
	HBODY h_driveeProxy = createArticulatedBody(bvh, t, false);
#if 0 //defined _DEBUG
	{
		std::cout << "Bounds:" << std::endl;
		int n_indent = 1;
		auto lam_onEnter = [&n_indent, &bvh = std::as_const(bvh), t](Bound b_this)
		{
			printBoundName(b_this, n_indent++);
			// BoundEQ(b_this, bvh, t);
			assert(BoundEQ(b_this, bvh, t));
		};
		auto lam_onLeave = [&n_indent](Bound b_this)
		{
			n_indent--;
		};
		Bound root = std::make_pair(bvh.root_joint(), h_driveeProxy);
		TraverseDFS_boundtree_recur(root, lam_onEnter, lam_onLeave);
	}
#endif
	HBODY h_drivee = createArticulatedBody(bvh, t, true);
#if defined _DEBUG
	{
		std::cout << "Bounds:" << std::endl;
		int n_indent = 1;
		auto lam_onEnter = [&n_indent, &bvh = std::as_const(bvh), t](Bound b_this)
		{
			printBoundName(b_this, n_indent++);
			assert(BoundResetAsBVHRest(b_this, bvh, t));
		};
		auto lam_onLeave = [&n_indent](Bound b_this)
		{
			n_indent--;
		};
		Bound root = std::make_pair(bvh.root_joint(), h_drivee);
		TraverseDFS_boundtree_recur(root, lam_onEnter, lam_onLeave);
	}
#endif
	HMOTIONNODE h_motion_driver = create_tree_motion_node(h_driver);
	HMOTIONNODE h_motion_driveeProxy = create_tree_motion_node(h_driveeProxy);
	bool sync_created = motion_sync_cnn_homo(h_motion_driver, h_motion_driveeProxy, FIRSTCHD);
	assert(sync_created && "homo sync should be created for 2 same-header-BVHs");

	HMOTIONNODE h_motion_drivee = create_tree_motion_node(h_drivee);
	sync_created = motion_sync_cnn_cross_c(h_motion_driveeProxy, h_motion_drivee, FIRSTCHD, NULL, 0);
	assert(sync_created && "cross sync should be created for 2 algined postures");

	bool pre_reset_header = true;

	bool header_resetted = false;
	if (pre_reset_header)
	{
		updateHeader(bvh, h_drivee);
		header_resetted = true;
	}

	for (int i_frame = 0
		; i_frame < n_frames
		; i_frame++)
	{
		pose_nonrecur(h_driver, bvh, i_frame, header_resetted);
		motion_sync(h_motion_driver);
		updateBVHAnim(h_drivee, bvh, i_frame, header_resetted);
	}

	if (!pre_reset_header)
		updateHeader(bvh, h_drivee);

	auto lam_onMoEnter = [](HMOTIONNODE node)
	{
	};
	auto lam_onMoLeave = [](HMOTIONNODE node)
	{
		destroy_tree_motion_node(node);
	};
	TraverseDFS_motree_nonrecur(h_motion_driver, lam_onMoEnter, lam_onMoLeave);

	auto lam_onBoEnter = [](HBODY node)
	{
	};
	auto lam_onBoLeave = [](HBODY node)
	{
		destroy_tree_body_node(node);
	};
	TraverseDFS_botree_nonrecur(h_driver, lam_onBoEnter, lam_onBoLeave);
	TraverseDFS_botree_nonrecur(h_driveeProxy, lam_onBoEnter, lam_onBoLeave);
	TraverseDFS_botree_nonrecur(h_drivee, lam_onBoEnter, lam_onBoLeave);

	return true;
}

HBODY create_tree_body_bvh_file(const wchar_t* path_src)
{
	std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
	auto path_src_c = converter.to_bytes(path_src);
	bvh11::BvhObject bvh(path_src_c);
	HBODY h_root = createArticulatedBody(bvh, -1, false);
	return h_root;
}

HBODY create_tree_body_bvh(HBVH hBvh)
{
	// bvh11::BvhObject* bvh = reinterpret_cast<bvh11::BvhObject*>(hBvh);
	bvh11::BvhObject* bvh = CAST_2P<HBVH, bvh11::BvhObject>(hBvh);
	HBODY h_root = createArticulatedBody(*bvh, -1, false);
	return h_root;
}

HBVH load_bvh(const wchar_t* path_src)
{
	std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
	auto path_src_c = converter.to_bytes(path_src);
	bvh11::BvhObject* bvh = new bvh11::BvhObject(path_src_c);
	return CAST_2HBVH( bvh);
}

void unload_bvh(HBVH hBvh)
{
	bvh11::BvhObject* bvh = CAST_2PBVH(hBvh);
	delete bvh;
}

unsigned int get_n_frames(HBVH hBvh)
{
	bvh11::BvhObject* bvh = CAST_2PBVH(hBvh);
	return bvh->frames();
}

bool ResetRestPose(const char* path_src, int frame, const char* path_dst)
{
	bvh11::BvhObject bvh(path_src);
	int frame_bvh11 = frame - 1;
	bool resetted = ResetRestPose(bvh, frame_bvh11);
	if (resetted)
			bvh.WriteBvhFile(path_dst);
	return resetted;
}

void pose_body(HBVH bvh, HBODY body, int i_frame)
{
	auto pBvh = CAST_2PBVH(bvh);
	pose_nonrecur(body, *pBvh, i_frame, false);
}

#pragma warning( pop )